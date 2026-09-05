// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/sqlite3_writer.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_compressor.hpp"
#include "bagwiz/io/message_compressor.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/sqlite3_helpers.hpp"
#include "env_tuning.hpp"                   // NOLINT(build/include_subdir) src-local shared header
#include "mcap_parallel_chunk_writer.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "parallel_message_compressor.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <sqlite3.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.sqlite3";
// Conservative batch size: keep transactions small enough that a crash loses
// at most a bounded number of messages but large enough to amortize commit
// overhead.
constexpr int kBatchSize = 1024;
// Cap on payload bytes pinned inside the MESSAGE-mode compression pool. The
// pipeline's own queue already bounds 128 MiB upstream; this keeps the pool
// from growing unboundedly when inserts (not compression) are the laggard.
constexpr std::uint64_t kPoolInFlightCap = 128ULL * 1024 * 1024;

// Transparent string hashing so per-message topic-id lookups can find() by
// string_view without materializing a std::string per message.
struct StringHash
{
  using is_transparent = void;
  size_t operator()(std::string_view v) const { return std::hash<std::string_view>{}(v); }
};
template <typename V>
using StringMap = std::unordered_map<std::string, V, StringHash, std::equal_to<>>;
// rosbag2 bagfile-information schema version written into the `metadata` table.
// Pinned to the humble baseline (and matched by write_metadata_yaml) so one
// output shape stays readable on every supported distro: rosbag2 reads older
// versions forward but not newer ones backward, and bagwiz hands its output to
// consumers whose distro it does not control. Raising this would make bags
// written under jazzy unopenable under humble.
constexpr int kMetadataVersion = 5;

// The compression setup resolved from CreateOptions, validated once at writer
// construction so a bad flag fails at open_write() rather than mid-bag.
struct Sqlite3Compression
{
  bool message_mode = false;  // rosbag2 compression_mode: MESSAGE (per-message zstd)
  bool file_mode = false;     // rosbag2 compression_mode: FILE (.db3.zstd envelope)
  int zstd_level = 0;         // 0 = ZSTD_defaultCLevel()
};

Sqlite3Compression resolve_sqlite3_compression(const CreateOptions & options)
{
  Sqlite3Compression out;

  std::string mode = options.sqlite3_compression_mode;
  std::string format = options.sqlite3_compression_format;
  if (mode.empty()) {
    mode = "none";
  }
  if (format.empty()) {
    format = "none";
  }

  if (format != "none" && format != "zstd") {
    throw std::runtime_error(
      "sqlite3 compression_format '" + options.sqlite3_compression_format +
      "' is not supported (rosbag2 defines only 'zstd' for sqlite3 storage)");
  }
  if (mode != "none" && mode != "message" && mode != "file") {
    throw std::runtime_error(
      "sqlite3 compression_mode '" + options.sqlite3_compression_mode +
      "' is not supported (expected \"none\", \"message\", or \"file\")");
  }
  if (mode == "none" && format != "none") {
    throw std::runtime_error(
      "sqlite3 compression_format '" + options.sqlite3_compression_format +
      "' requires compression_mode \"message\" or \"file\"");
  }
  if (mode != "none" && format == "none") {
    throw std::runtime_error(
      "sqlite3 compression_mode \"" + mode + "\" requires compression_format 'zstd'");
  }

  out.message_mode = mode == "message";
  out.file_mode = mode == "file";
  // Throws on an unknown level name — that is the validation we want here.
  out.zstd_level = zstd_level_from_name(options.sqlite3_compression_level);
  return out;
}

void exec_or_throw(sqlite3 * db, const char * sql)
{
  char * err = nullptr;
  const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
  if (rc != SQLITE_OK) {
    std::string msg = err ? err : "<no message>";
    sqlite3_free(err);
    throw std::runtime_error(std::string("sqlite exec failed: ") + msg + " (sql=" + sql + ")");
  }
  sqlite3_free(err);
}

// ---------------------------------------------------------------------------
// Single .db3 file writer.
// ---------------------------------------------------------------------------
class SqliteFileWriter final : public BagWriter
{
public:
  explicit SqliteFileWriter(const std::filesystem::path & path, const CreateOptions & options)
  : db_(sqlite_open_or_throw(
      path.string(), SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX,
      "sqlite3 open")),
    shard_rel_(path.filename().string()),
    compression_(resolve_sqlite3_compression(options))
  {
    if (compression_.file_mode) {
      // FILE mode wraps the finished shard in a whole-database envelope,
      // which is the directory writer's job: the envelope renames the shard to
      // .db3.zstd, and metadata.yaml is what records that name. The
      // directory writer never
      // passes "file" down here (it clears the mode on the inner options and
      // compresses the finished shard itself), so reaching this branch means
      // a standalone single-file output was requested.
      throw std::runtime_error(
        "sqlite3 compression_mode \"file\" requires a directory-layout output: the "
        "envelope renames the shard to .db3.zstd and metadata.yaml is what records "
        "that name; name an extension-less output directory instead of a .db3 file");
    }
    if (compression_.message_mode) {
      // Reached only from the directory writer's inner shard (declared as
      // MESSAGE-mode in the directory's metadata.yaml); create_sqlite3_file()
      // rejects MESSAGE mode for standalone single-file outputs, whose
      // reader has no metadata.yaml to learn the compression from.
      // Per-message compression is the write path's CPU bottleneck, so with
      // workers available it moves onto a pool that the insert loop drains in
      // submission order; the knob's 0/1 serial value keeps the historical
      // inline compressor.
      const int workers = resolve_write_threads();
      if (workers >= 2) {
        pool_.emplace(workers, compression_.zstd_level);
      } else {
        compressor_.emplace("zstd", compression_.zstd_level);
      }
    }

    // page_size must come first: SQLite only honours it while the database is
    // still empty, so any pragma or statement that writes a page locks in the
    // 4 KiB default. Rosbag payloads are large BLOBs, and at 4 KiB a 2 MB point
    // cloud spills across ~500 chained overflow pages versus ~64 at 32 KiB.
    exec_or_throw(
      db_.get(),
      ("PRAGMA page_size = " + std::to_string(resolve_db3_page_size(kLogger)) + ";").c_str());

    // Write-side tuning. journal_mode=MEMORY keeps crash-consistency at the
    // cost of some durability; OFF would be faster but leaves a corrupt bag
    // on crash. bagwiz writes new bags so losing one on crash is acceptable,
    // but MEMORY is the better default.
    exec_or_throw(db_.get(), "PRAGMA journal_mode = MEMORY;");
    exec_or_throw(db_.get(), "PRAGMA synchronous = OFF;");
    exec_or_throw(db_.get(), "PRAGMA temp_store = MEMORY;");
    exec_or_throw(db_.get(), "PRAGMA cache_size = -65536;");

    create_schema();
    prepare_insert_stmt();
    begin_transaction();
  }

  ~SqliteFileWriter() override
  {
    if (!closed_) {
      try {
        SqliteFileWriter::close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "SqliteFileWriter close failed: %s", e.what());
      } catch (...) {
        // Never throw from destructor.
      }
    }
  }

  SqliteFileWriter(const SqliteFileWriter &) = delete;
  SqliteFileWriter & operator=(const SqliteFileWriter &) = delete;
  SqliteFileWriter(SqliteFileWriter &&) = delete;
  SqliteFileWriter & operator=(SqliteFileWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    if (topic_to_id_.count(topic.name) != 0U) {
      return;  // already declared
    }

    auto stmt = sqlite_prepare_or_throw(
      db_.get(),
      "INSERT INTO topics(name, type, serialization_format, offered_qos_profiles, "
      "type_description_hash) VALUES (?, ?, ?, ?, ?);");
    sqlite3_bind_text(stmt.get(), 1, topic.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 2, topic.type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 3, topic.serialization_format.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 4, topic.offered_qos_profiles.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 5, topic.type_description_hash.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
      throw std::runtime_error("topic insert failed: " + sqlite_errmsg(db_.get()));
    }

    topic_to_id_[topic.name] = sqlite3_last_insert_rowid(db_.get());
    topics_.push_back(topic);
    topic_counts_.push_back(0);

    // Insert message_definitions row once per type (deduped). Iron+ rosbag2
    // readers query this table directly for self-description; the row is
    // optional (rows with empty topic_type are ignored by the upstream
    // reader, which simply skips encoded_message_definition lookup). We
    // emit a row whenever we have a non-empty schema_text so the bag stays
    // self-describing across a repack.
    if (!topic.schema_text.empty() && type_to_msgdef_id_.count(topic.type) == 0U) {
      auto def_stmt = sqlite_prepare_or_throw(
        db_.get(),
        "INSERT INTO message_definitions("
        "  topic_type, encoding, encoded_message_definition, type_description_hash) "
        "VALUES (?, ?, ?, ?);");
      const std::string encoding =
        topic.schema_encoding.empty() ? std::string("ros2msg") : topic.schema_encoding;
      sqlite3_bind_text(def_stmt.get(), 1, topic.type.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(def_stmt.get(), 2, encoding.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(def_stmt.get(), 3, topic.schema_text.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(
        def_stmt.get(), 4, topic.type_description_hash.c_str(), -1, SQLITE_TRANSIENT);
      if (sqlite3_step(def_stmt.get()) != SQLITE_DONE) {
        throw std::runtime_error("message_definitions insert failed: " + sqlite_errmsg(db_.get()));
      }
      type_to_msgdef_id_[topic.type] = sqlite3_last_insert_rowid(db_.get());
    }
  }

  void write(
    // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    if (pool_) {
      write_pooled(topic, timestamp_ns, payload, nullptr);
      return;
    }

    const int64_t topic_id = lookup_topic(topic);
    // MESSAGE mode stores each payload as a bare zstd frame (rosbag2
    // compression plugin contract). SQLITE_STATIC stays correct in both
    // branches: the bound bytes must only outlive the sqlite3_step() below,
    // and the compressor's internal buffer is valid until the next write().
    const std::span<const std::byte> stored =
      compressor_.has_value() ? compressor_->compress(payload) : payload;
    insert_row(topic_id, timestamp_ns, stored);
    update_accounting(topic_id, timestamp_ns);
  }

  // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
  void write_frozen(std::string_view topic, FrozenMessage msg) override
  {
    // The pool pins the payload through the shared owner instead of copying
    // it. Without an owner the span's lifetime is call-scoped, so it takes
    // the spill copy inside write_pooled (or the serial path below).
    if (pool_) {
      write_pooled(topic, msg.timestamp_ns, msg.payload, std::move(msg.owner));
      return;
    }
    write(topic, msg.timestamp_ns, msg.payload);
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    if (pool_) {
      // Drain in submission order, then join the workers. A latched
      // compression error throws from take_blocking()/finish().
      ParallelMessageCompressor::Job job;
      while (!pool_->empty()) {
        pool_->take_blocking(job);
        insert_row(job.topic_id, job.timestamp_ns, job.compressed);
      }
      pool_->finish();
    }
    commit_transaction();
    write_metadata_row();
    closed_ = true;
  }

  // The message accounting gathered during the write, in the shape both
  // metadata consumers need: the embedded `metadata` row here and the
  // directory writer's metadata.yaml. Kept in one place so the two can never
  // disagree about the same bag.
  //
  // `shard_relative_path` is this file's own name, which for a directory bag's
  // shard is exactly the name metadata.yaml lists.
  [[nodiscard]] MetadataYamlInfo summary() const
  {
    MetadataYamlInfo info;
    info.storage_identifier = "sqlite3";
    info.topics = topics_;
    for (std::size_t i = 0; i < topics_.size(); ++i) {
      info.per_topic_counts[topics_[i].name] = topic_counts_[i];
    }
    info.total_messages = total_messages_;
    info.start_ns = start_ns_;
    info.end_ns = end_ns_;
    info.shard_relative_path = shard_rel_;
    if (compression_.message_mode) {
      // Declared in both the embedded `metadata` row and the directory's
      // metadata.yaml, so the two descriptions of this bag cannot drift.
      info.compression_format = "zstd";
      info.compression_mode = "message";
    }
    return info;
  }

private:
  int64_t lookup_topic(std::string_view topic) const
  {
    const auto it = topic_to_id_.find(topic);
    if (it == topic_to_id_.end()) {
      throw std::runtime_error(
        "sqlite3 write on undeclared topic: " + std::string(topic) +
        " (call declare_topic() first)");
    }
    return it->second;
  }

  void insert_row(int64_t topic_id, int64_t timestamp_ns, std::span<const std::byte> stored)
  {
    sqlite3_bind_int64(insert_stmt_.get(), 1, topic_id);
    sqlite3_bind_int64(insert_stmt_.get(), 2, timestamp_ns);
    // SQLITE_STATIC is correct: the bound bytes must only outlive the
    // sqlite3_step() below, and every producer (the serial compressor's
    // internal buffer, a drained pool job's vector, the caller's own span)
    // outlives it.
    sqlite3_bind_blob(
      insert_stmt_.get(), 3, stored.data(), static_cast<int>(stored.size()), SQLITE_STATIC);
    if (sqlite3_step(insert_stmt_.get()) != SQLITE_DONE) {
      throw std::runtime_error("message insert failed: " + sqlite_errmsg(db_.get()));
    }
    sqlite3_reset(insert_stmt_.get());

    if (++pending_in_tx_ >= kBatchSize) {
      commit_transaction();
      begin_transaction();
      pending_in_tx_ = 0;
    }
  }

  void update_accounting(int64_t topic_id, int64_t timestamp_ns)
  {
    // Topic rowids are assigned sequentially from 1 on this fresh table, so
    // the count vector is indexed by rowid - 1 (declare order).
    ++topic_counts_[static_cast<std::size_t>(topic_id - 1)];
    ++total_messages_;
    if (timestamp_ns < start_ns_) {
      start_ns_ = timestamp_ns;
    }
    if (timestamp_ns > end_ns_) {
      end_ns_ = timestamp_ns;
    }
  }

  // MESSAGE-mode write through the compression pool. Inserts happen on this
  // (the caller) thread in submission order; only compression runs on workers.
  void write_pooled(
    // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload,
    std::shared_ptr<const void> owner)
  {
    const int64_t topic_id = lookup_topic(topic);

    // The pool defers compression past this call, so the payload must be
    // pinned. A caller that transferred no owner (plain write()) pays one
    // spill copy. Build the span and the owner in separate statements:
    // mixing std::move(spill) into the submit() call expression would let the
    // span read the owner after the move (unspecified argument order).
    std::shared_ptr<const void> pinned = std::move(owner);
    std::span<const std::byte> bytes = payload;
    FrozenMessage spill;
    if (!pinned && !payload.empty()) {
      spill = own_payload(std::vector<std::byte>(payload.begin(), payload.end()));
      bytes = spill.payload;
      pinned = spill.owner;
    }

    // Keep the in-flight window under the cap by inserting the oldest
    // completed jobs first. An empty pool always admits: a single message
    // larger than the cap is admitted whole rather than deadlocking.
    while (!pool_->empty() && pool_->in_flight_bytes() + bytes.size() > kPoolInFlightCap) {
      ParallelMessageCompressor::Job job;
      pool_->take_blocking(job);
      insert_row(job.topic_id, job.timestamp_ns, job.compressed);
    }
    pool_->submit(topic_id, timestamp_ns, bytes, std::move(pinned));
    ParallelMessageCompressor::Job job;
    while (pool_->try_take(job)) {
      insert_row(job.topic_id, job.timestamp_ns, job.compressed);
    }
    update_accounting(topic_id, timestamp_ns);
  }

  // rosbag2 iron+ stores the bag summary in the `metadata` table so a
  // single-file .db3 is self-describing without a metadata.yaml beside it.
  // Filling it lets compute_stats() answer per-topic counts without scanning
  // the BLOB-laden messages rows — the speedup the old (topic_id, timestamp)
  // index used to provide, now without steering any other reader's query plan.
  //
  // Humble ignores this table entirely (its plugin creates it but has no SQL
  // that reads or writes it), so the row is inert there; jazzy+ parses it at
  // open time. That parse is why the YAML must come from the shared emitter:
  // a structure that disagrees with its declared `version` makes jazzy reject
  // the file outright rather than just misreport it.
  void write_metadata_row()
  {
    const std::string yaml = emit_metadata_yaml_body(summary());
    auto stmt = sqlite_prepare_or_throw(
      db_.get(), "INSERT INTO metadata(metadata_version, metadata) VALUES (?, ?);");
    sqlite3_bind_int(stmt.get(), 1, kMetadataVersion);
    sqlite3_bind_text(stmt.get(), 2, yaml.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
      throw std::runtime_error("metadata insert failed: " + sqlite_errmsg(db_.get()));
    }
  }

  void create_schema()
  {
    // Mirrors the rosbag2 sqlite3 plugin schema_version=4 layout
    // (Iron / Jazzy):
    //   - `topics` carries `type_description_hash` so the bag advertises
    //     its RIHS type identity to readers that care.
    //   - `message_definitions` stores per-type self-description so
    //     readers don't need a local typestore overlay sourced.
    // Older rosbag2 readers (Humble v3) tolerate the extra column and
    // unused table — they only SELECT the columns they know about. We
    // never write the v3-shaped schema; callers that need true v3
    // compatibility can decompose with rosbag2's reindex tool.
    exec_or_throw(
      db_.get(),
      "CREATE TABLE IF NOT EXISTS schema("
      "  schema_version INTEGER PRIMARY KEY,"
      "  ros_distro TEXT);"
      "CREATE TABLE IF NOT EXISTS metadata("
      "  id INTEGER PRIMARY KEY,"
      "  metadata_version INTEGER,"
      "  metadata TEXT);"
      "CREATE TABLE IF NOT EXISTS topics("
      "  id INTEGER PRIMARY KEY,"
      "  name TEXT NOT NULL,"
      "  type TEXT NOT NULL,"
      "  serialization_format TEXT NOT NULL,"
      "  offered_qos_profiles TEXT NOT NULL,"
      "  type_description_hash TEXT NOT NULL DEFAULT '');"
      "CREATE TABLE IF NOT EXISTS message_definitions("
      "  id INTEGER PRIMARY KEY,"
      "  topic_type TEXT NOT NULL,"
      "  encoding TEXT NOT NULL,"
      "  encoded_message_definition TEXT NOT NULL,"
      "  type_description_hash TEXT NOT NULL DEFAULT '');"
      "CREATE TABLE IF NOT EXISTS messages("
      "  id INTEGER PRIMARY KEY,"
      "  topic_id INTEGER NOT NULL,"
      "  timestamp INTEGER NOT NULL,"
      "  data BLOB NOT NULL);"
      "CREATE INDEX IF NOT EXISTS timestamp_idx ON messages (timestamp ASC);");

    // Iron / Jazzy schema_version. Older readers ignore unknown values.
    exec_or_throw(
      db_.get(), "INSERT OR IGNORE INTO schema(schema_version, ros_distro) VALUES (4, '');");
  }

  void prepare_insert_stmt()
  {
    insert_stmt_ = sqlite_prepare_or_throw(
      db_.get(), "INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?);");
  }

  void begin_transaction() { exec_or_throw(db_.get(), "BEGIN TRANSACTION;"); }

  void commit_transaction() { exec_or_throw(db_.get(), "COMMIT;"); }

  SqlitePtr db_;
  SqliteStmtPtr insert_stmt_;
  std::string shard_rel_;
  Sqlite3Compression compression_;
  // Present only in MESSAGE mode; compresses each payload before the insert.
  // With write workers available the pool owns compression instead and the
  // insert loop drains it in submission order.
  std::optional<MessageCompressor> compressor_;
  std::optional<ParallelMessageCompressor> pool_;
  StringMap<int64_t> topic_to_id_;
  // Message accounting for the embedded metadata row (see summary()).
  std::vector<TopicInfo> topics_;
  // Per-topic message counts indexed by topic rowid - 1 (declare order on a
  // fresh table); summary() keys them by name for MetadataYamlInfo.
  std::vector<int64_t> topic_counts_;
  int64_t total_messages_ = 0;
  int64_t start_ns_ = std::numeric_limits<int64_t>::max();
  int64_t end_ns_ = std::numeric_limits<int64_t>::min();
  // Tracks message_definitions rows already written, keyed by ROS 2 type
  // name. Each type gets exactly one row regardless of how many topics
  // share it.
  std::unordered_map<std::string, int64_t> type_to_msgdef_id_;
  int pending_in_tx_ = 0;
  bool closed_ = false;
};

// ---------------------------------------------------------------------------
// Directory writer: single .db3 shard + metadata.yaml.
// ---------------------------------------------------------------------------
class SqliteDirectoryWriter final : public BagWriter
{
public:
  SqliteDirectoryWriter(const std::filesystem::path & dir, const CreateOptions & options)
  : dir_(dir), options_(options), compression_(resolve_sqlite3_compression(options))
  {
    std::filesystem::create_directories(dir);
    const auto stem = dir.filename().string();
    shard_rel_ = stem + "_0.db3";

    // FILE mode is the directory writer's own concern: the inner shard is
    // written plain and wrapped in the .db3.zstd envelope at close(), so the
    // file writer below always sees "none" for that mode. MESSAGE mode is
    // passed down — it changes what the shard stores per message.
    CreateOptions inner_options = options;
    if (compression_.file_mode) {
      inner_options.sqlite3_compression_mode = "none";
      inner_options.sqlite3_compression_format = "none";
    }
    inner_ = std::make_unique<SqliteFileWriter>(dir_ / shard_rel_, inner_options);
  }

  ~SqliteDirectoryWriter() override
  {
    if (!closed_) {
      try {
        SqliteDirectoryWriter::close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "SqliteDirectoryWriter close failed: %s", e.what());
      } catch (...) {
      }
    }
  }

  SqliteDirectoryWriter(const SqliteDirectoryWriter &) = delete;
  SqliteDirectoryWriter & operator=(const SqliteDirectoryWriter &) = delete;
  SqliteDirectoryWriter(SqliteDirectoryWriter &&) = delete;
  SqliteDirectoryWriter & operator=(SqliteDirectoryWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override { inner_->declare_topic(topic); }

  void write(
    // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    inner_->write(topic, timestamp_ns, payload);
  }

  // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
  void write_frozen(std::string_view topic, FrozenMessage msg) override
  {
    inner_->write_frozen(topic, std::move(msg));
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    // Take the summary before close() so metadata.yaml and the shard's own
    // embedded `metadata` row are emitted from one accumulator — they describe
    // the same bag and must not be able to drift.
    MetadataYamlInfo info = inner_->summary();
    inner_->close();

    if (compression_.file_mode) {
      // rosbag2 compression_mode: FILE — wrap the finished shard in a
      // whole-database zstd envelope and point metadata.yaml at it. The
      // shard's embedded `metadata` row stays a plain-storage declaration:
      // it describes the .db3 bytes as written, which is what a reader sees
      // after expanding the envelope.
      const auto plain_shard = dir_ / shard_rel_;
      shard_rel_ += ".zstd";
      compress_file_to_zstd(plain_shard, dir_ / shard_rel_, compression_.zstd_level);
      std::error_code ec;
      std::filesystem::remove(plain_shard, ec);
      if (ec) {
        throw std::runtime_error(
          "failed to remove plain shard after zstd envelope compression: " + plain_shard.string() +
          ": " + ec.message());
      }
      info.compression_format = "zstd";
      info.compression_mode = "file";
      info.shard_relative_path = shard_rel_;
    }

    write_metadata_yaml(dir_, info);
    closed_ = true;
  }

private:
  std::filesystem::path dir_;
  CreateOptions options_;
  std::string shard_rel_;
  Sqlite3Compression compression_;
  std::unique_ptr<SqliteFileWriter> inner_;
  bool closed_ = false;
};

}  // namespace

std::unique_ptr<BagWriter> create_sqlite3_file(
  const std::filesystem::path & path, const CreateOptions & options)
{
  // A bare .db3 has no metadata.yaml beside it, so a single-file output can
  // only ever be plain storage. bagwiz itself would read a MESSAGE-mode one
  // back correctly — the declaration is in the file's own `metadata` table —
  // but rosbag2 picks its reader from metadata.yaml alone, so it would hand
  // the stored zstd frames straight to the caller and report success. FILE
  // mode's envelope additionally needs the .db3.zstd name recorded for it.
  // Reject both here so the failure surfaces at open_write().
  const auto compression = resolve_sqlite3_compression(options);
  if (compression.message_mode || compression.file_mode) {
    throw std::runtime_error(
      "sqlite3 compression requires a directory-layout output: rosbag2 only "
      "decompresses when a metadata.yaml declares the mode, so a bare .db3 would "
      "read back as raw zstd frames with no error; name an extension-less output "
      "directory instead of a .db3 file");
  }
  return std::make_unique<SqliteFileWriter>(path, options);
}

std::unique_ptr<BagWriter> create_sqlite3_directory(
  const std::filesystem::path & dir, const CreateOptions & options)
{
  return std::make_unique<SqliteDirectoryWriter>(dir, options);
}

}  // namespace bagwiz::io::detail
