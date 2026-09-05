// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "mcap_parallel_chunk_writer.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "writeback_window.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/writer.hpp>

#include <algorithm>
#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{

constexpr const char * kLogger = "bagwiz.io.mcap";

// IWritable that appends to a byte vector: the static libmcap record writers
// serialize Message/Schema/Channel records into it.
class VectorWritable final : public mcap::IWritable
{
public:
  void end() override {}
  uint64_t size() const override { return buffer_.size(); }

  std::vector<std::byte> buffer_;

protected:
  void handleWrite(const std::byte * data, uint64_t size) override
  {
    buffer_.insert(buffer_.end(), data, data + size);
  }
};

// mcap Message record prefix: opcode + record length + channelId + sequence +
// logTime + publishTime, serialized little-endian the same way libmcap's
// record writers do (31 bytes; the payload follows it inside the chunk).
constexpr std::size_t kMessageHeaderBytes = 1 + 8 + 2 + 4 + 8 + 8;

// Serialize the 31-byte Message record prefix. The caller-side fields match
// what McapFileWriter sets on every message: sequence 0, publishTime ==
// logTime.
void serialize_message_header(
  std::array<std::byte, kMessageHeaderBytes> & out, mcap::ChannelId channel,
  mcap::Timestamp log_time, std::uint64_t payload_size)
{
  const std::uint8_t op = static_cast<std::uint8_t>(mcap::OpCode::Message);
  const std::uint64_t record_len = 2 + 4 + 8 + 8 + payload_size;
  const std::uint32_t sequence = 0;
  std::byte * p = out.data();
  const auto put = [&p](const auto & v) {
    std::memcpy(p, &v, sizeof(v));
    p += sizeof(v);
  };
  put(op);
  put(record_len);
  put(channel);
  put(sequence);
  put(log_time);
  put(log_time);  // publishTime
}

// One message inside a chunk: the serialized record prefix plus a gather
// reference to the payload bytes. `owner` pins the payload's backing store
// (a reader chunk buffer, a transform's output buffer, or a spill copy) until
// the compression worker has consumed the segment.
struct Segment
{
  std::array<std::byte, kMessageHeaderBytes> header;
  const std::byte * payload = nullptr;
  std::uint64_t payload_size = 0;
  std::shared_ptr<const void> owner;
};

// One unit of output, in emission order. A records job carries serialized
// top-level Schema/Channel records verbatim; a chunk job carries one chunk's
// messages as gather segments and is compressed by a worker before the
// sequencer writes it.
struct Job
{
  std::uint64_t seq = 0;
  bool is_chunk = false;
  bool claimed = false;  // a worker owns this chunk's compression
  bool ready = false;    // bytes/compressed are final and may be written
  std::string error;     // worker-side failure, latched by the sequencer

  // Records job payload.
  std::vector<std::byte> bytes;

  // Chunk job payload.
  std::vector<Segment> segments;
  std::uint64_t uncompressed_size = 0;  // sum of segment bytes
  std::vector<std::byte> compressed;
  std::string compression;  // codec chosen for THIS chunk ("" on fallback)
  mcap::Timestamp start_time = 0;
  mcap::Timestamp end_time = 0;
  std::uint64_t message_count = 0;
  std::map<mcap::ChannelId, std::vector<std::pair<mcap::Timestamp, mcap::ByteOffset>>> index;
};

}  // namespace

class ParallelChunkMcapWriter::Impl
{
public:
  Impl(
    const std::filesystem::path & path, std::string compression, mcap::CompressionLevel level,
    std::uint64_t chunk_size, int num_threads)
  : compression_(std::move(compression)),
    chunk_size_(chunk_size),
    max_inflight_(static_cast<std::size_t>(std::max(num_threads, 1)) + 2)
  {
    if (const auto status = out_.open(path.string()); !status.ok()) {
      throw std::runtime_error(
        "mcap writer open failed for " + path.string() + ": " + status.message);
    }
    // Writeback window over the output file; note_offset() calls from
    // the sequencer keep its dirty backlog bounded. Built only after open()
    // because the window opens its own management fd on the file.
    writeback_window_.emplace(path, resolve_writeback_interval_bytes(kLogger));
    mcap::McapWriter::writeMagic(out_);
    mcap::McapWriter::write(out_, mcap::Header{"ros2", "bagwiz"});

    // One reusable chunk encoder per worker, built here so an allocation
    // failure surfaces from the constructor instead of inside a thread body.
    // (A deque because mcap::ZStdWriter is neither copyable nor movable.)
    const int workers = std::max(num_threads, 1);
    for (int i = 0; i < workers; ++i) {
      encoders_.emplace_back(level, chunk_size_);
    }
    workers_.reserve(static_cast<std::size_t>(workers));
    for (int i = 0; i < workers; ++i) {
      workers_.emplace_back([this, i]() { worker_loop(encoders_[static_cast<std::size_t>(i)]); });
    }
    sequencer_ = std::jthread([this]() { sequencer_loop(); });
  }

  ~Impl()
  {
    if (!closed_) {
      try {
        close();
      } catch (...) {
        // Destructors must not throw. The output is incomplete either way.
      }
    }
  }

  void write_schema(const mcap::Schema & schema)
  {
    VectorWritable w;
    mcap::McapWriter::write(w, schema);
    schemas_.push_back(schema);
    enqueue_records(std::move(w.buffer_));
  }

  void write_channel(const mcap::Channel & channel)
  {
    VectorWritable w;
    mcap::McapWriter::write(w, channel);
    channels_.push_back(channel);
    enqueue_records(std::move(w.buffer_));
  }

  void write_message(const mcap::Message & message)
  {
    // The caller transfers no ownership, so the payload may dangle once this
    // call returns while compression runs later on a worker. Copy it into an
    // owned spill buffer and feed the same gather path. This is the rare
    // entry (direct write() callers: tests, the sequential backend, command
    // code with local buffers); the pipeline's frozen messages arrive through
    // the owning overload below with no copy at all.
    const std::span<const std::byte> payload(message.data, message.dataSize);
    if (payload.empty()) {
      // No bytes to pin: skip the spill copy (and its null-iterator corner).
      write_message_owned(message.channelId, message.logTime, payload, nullptr);
      return;
    }
    auto spill = std::make_shared<std::vector<std::byte>>(payload.begin(), payload.end());
    // Build the span and the owner in separate statements: constructing the
    // by-value shared_ptr parameter directly from std::move(spill) in the same
    // call expression would let the span read spill AFTER it was moved from
    // (argument evaluation order is unspecified).
    const std::span<const std::byte> owned(spill->data(), spill->size());
    std::shared_ptr<const void> owner = std::move(spill);
    write_message_owned(message.channelId, message.logTime, owned, std::move(owner));
  }

  void write_message_owned(
    mcap::ChannelId channel, mcap::Timestamp log_time, std::span<const std::byte> payload,
    std::shared_ptr<const void> owner)
  {
    throw_if_failed();
    Segment seg;
    serialize_message_header(seg.header, channel, log_time, payload.size());
    seg.payload = payload.data();
    seg.payload_size = payload.size();
    seg.owner = std::move(owner);
    staging_index_[channel].emplace_back(log_time, staging_size_);
    staging_size_ += seg.header.size() + seg.payload_size;
    staging_times_min_ = std::min(staging_times_min_, log_time);
    staging_times_max_ = std::max(staging_times_max_, log_time);
    ++staging_count_;
    staging_segments_.push_back(std::move(seg));
    if (staging_size_ >= chunk_size_) {
      flush_staging();
    }
  }

  void close()
  {
    if (closed_) {
      return;
    }
    if (staging_count_ > 0) {
      flush_staging();
    }

    // Wait for the sequencer to write every queued job, then stop the
    // threads. Workers exit on stop_ only once no unclaimed chunk remains,
    // so draining first keeps the shutdown order deterministic.
    {
      std::unique_lock lock(mutex_);
      cv_.wait(lock, [&] { return jobs_.empty() || !error_.empty(); });
      stop_ = true;
    }
    cv_.notify_all();
    if (sequencer_.joinable()) {
      sequencer_.join();
    }
    for (auto & w : workers_) {
      if (w.joinable()) {
        w.join();
      }
    }
    throw_if_failed();

    mcap::McapWriter::write(out_, mcap::DataEnd{0});
    write_summary_section();
    out_.end();
    // Flush the bounded dirty remainder of the output.
    writeback_window_->finish();
    closed_ = true;
  }

private:
  // mcap::IChunkWriter is final per codec; keep both in one holder so the
  // worker picks by codec name. Only the selected one is ever used.
  struct Encoder
  {
    mcap::ZStdWriter zstd;
    mcap::LZ4Writer lz4;
    Encoder(mcap::CompressionLevel level, std::uint64_t chunk_size)
    : zstd(level, chunk_size), lz4(level, chunk_size)
    {
    }
  };

  void enqueue_records(std::vector<std::byte> bytes)
  {
    throw_if_failed();
    wait_for_room();
    auto job = std::make_unique<Job>();
    job->seq = next_seq_++;
    job->ready = true;
    job->bytes = std::move(bytes);
    {
      std::lock_guard lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    cv_.notify_all();
  }

  // Hand the staged segments to the pool as the next chunk job and start a
  // fresh staging set. Caller thread only.
  void flush_staging()
  {
    wait_for_room();
    auto job = std::make_unique<Job>();
    job->seq = next_seq_++;
    job->is_chunk = true;
    job->segments = std::move(staging_segments_);
    job->uncompressed_size = staging_size_;
    job->start_time = staging_times_min_;
    job->end_time = staging_times_max_;
    job->message_count = staging_count_;
    job->index = std::move(staging_index_);
    {
      std::lock_guard lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    cv_.notify_all();

    staging_segments_.clear();
    staging_index_.clear();
    staging_size_ = 0;
    staging_times_min_ = mcap::MaxTime;
    staging_times_max_ = 0;
    staging_count_ = 0;
  }

  // Backpressure: bound the queued jobs so a fast reader cannot outrun the
  // compressor pool by more than max_inflight_ chunk buffers.
  void wait_for_room()
  {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [&] { return jobs_.size() < max_inflight_ || !error_.empty(); });
    if (!error_.empty()) {
      throw std::runtime_error(error_);
    }
  }

  void throw_if_failed() const
  {
    std::lock_guard lock(mutex_);
    if (!error_.empty()) {
      throw std::runtime_error(error_);
    }
  }

  std::vector<std::byte> take_pooled_buffer()
  {
    std::lock_guard lock(mutex_);
    if (buffer_pool_.empty()) {
      return {};
    }
    auto buf = std::move(buffer_pool_.back());
    buffer_pool_.pop_back();
    return buf;
  }

  void recycle(std::vector<std::byte> && buf)
  {
    if (buf.capacity() == 0) {
      return;
    }
    buf.clear();
    std::lock_guard lock(mutex_);
    // Bound the pool: every chunk returns its compressed buffer, but a worker
    // consumes only one per chunk — an unbounded pool would grow by
    // ~compressed-chunk-size per chunk written. Buffers past the cap are
    // freed instead.
    if (buffer_pool_.size() >= max_inflight_ * 2) {
      return;
    }
    buffer_pool_.push_back(std::move(buf));
  }

  void worker_loop(Encoder & encoder)
  {
    while (true) {
      Job * job = nullptr;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [&] {
          return (stop_ && jobs_.empty()) || !error_.empty() || find_unclaimed() != nullptr;
        });
        if ((stop_ && jobs_.empty()) || !error_.empty()) {
          return;
        }
        job = find_unclaimed();
        job->claimed = true;
      }

      // A chunk job's fields are owned by the queue and touched by exactly
      // one thread at a time: the caller until `claimed`, this worker until
      // `ready`, the sequencer afterwards.
      try {
        compress_chunk(*job, encoder);
      } catch (const std::exception & e) {
        job->error = std::string("mcap chunk compression failed: ") + e.what();
      }

      {
        std::lock_guard lock(mutex_);
        job->ready = true;
        if (!job->error.empty() && error_.empty()) {
          error_ = job->error;
        }
      }
      cv_.notify_all();
    }
  }

  Job * find_unclaimed()
  {
    for (auto & job : jobs_) {
      if (job->is_chunk && !job->claimed) {
        return job.get();
      }
    }
    return nullptr;
  }

  // Compress one chunk one-shot over its gather segments. Each chunk is
  // compressed single-threaded, so the output bytes depend only on the input,
  // never on the worker count — output is byte-identical for any
  // BAGWIZ_WRITE_THREADS >= 2. Mirrors libmcap's forceCompression=false rule:
  // a chunk that does not shrink is stored uncompressed.
  void compress_chunk(Job & job, Encoder & encoder)
  {
    mcap::IChunkWriter * w = compression_ == "zstd"
                               ? static_cast<mcap::IChunkWriter *>(&encoder.zstd)
                               : static_cast<mcap::IChunkWriter *>(&encoder.lz4);
    w->clear();
    for (const auto & seg : job.segments) {
      w->write(seg.header.data(), seg.header.size());
      if (seg.payload_size != 0) {
        w->write(seg.payload, seg.payload_size);
      }
    }
    w->end();
    // The encoder's internal buffer now holds every byte, so the segments —
    // and the reader buffers / spill copies they pin — can go immediately.
    job.segments.clear();
    const std::uint64_t compressed_size = w->compressedSize();
    if (compressed_size >= job.uncompressed_size) {
      job.compression.clear();
      job.compressed = take_pooled_buffer();
      job.compressed.assign(w->data(), w->data() + job.uncompressed_size);
      return;
    }
    const std::byte * data = w->compressedData();
    job.compression = compression_;
    job.compressed = take_pooled_buffer();
    job.compressed.assign(data, data + compressed_size);
  }

  void sequencer_loop()
  {
    while (true) {
      std::unique_ptr<Job> job;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [&] {
          return !error_.empty() || (stop_ && jobs_.empty()) ||
                 (!jobs_.empty() && jobs_.front()->ready);
        });
        if (!error_.empty()) {
          // Do NOT clear jobs_: a worker may still hold a raw pointer into a
          // claimed job it is compressing. The queue is left for the Impl
          // destructor after close() has joined every worker.
          cv_.notify_all();
          return;
        }
        if (jobs_.empty()) {
          cv_.notify_all();
          return;  // stop_ with a drained queue
        }
        job = std::move(jobs_.front());
        jobs_.pop_front();
      }
      cv_.notify_all();  // backpressure waiters: a slot freed

      if (job->is_chunk) {
        write_chunk(*job);
        recycle(std::move(job->compressed));
      } else {
        out_.write(job->bytes.data(), job->bytes.size());
      }
      writeback_window_->note_offset(out_.size());
    }
  }

  // Write one Chunk record followed by its per-channel MessageIndex records,
  // and accumulate the ChunkIndex/statistics the summary section needs.
  void write_chunk(Job & job)
  {
    mcap::Chunk chunk;
    chunk.messageStartTime = job.start_time;
    chunk.messageEndTime = job.end_time;
    chunk.uncompressedSize = job.uncompressed_size;
    // Chunk CRCs cost a pass over every written byte and common readers do
    // not validate them (see McapFileWriter); skip, keep the summary CRC.
    chunk.uncompressedCrc = 0;
    chunk.compression = job.compression;
    chunk.compressedSize = job.compressed.size();
    chunk.records = job.compressed.data();

    const std::uint64_t chunk_start = out_.size();
    mcap::McapWriter::write(out_, chunk);
    const std::uint64_t index_start = out_.size();

    mcap::ChunkIndex ci;
    ci.messageStartTime = job.start_time;
    ci.messageEndTime = job.end_time;
    ci.chunkStartOffset = chunk_start;
    ci.chunkLength = index_start - chunk_start;
    for (const auto & index_entry : job.index) {
      ci.messageIndexOffsets[index_entry.first] = out_.size();
      mcap::MessageIndex mi;
      mi.channelId = index_entry.first;
      mi.records = index_entry.second;
      mcap::McapWriter::write(out_, mi);
      channel_counts_[index_entry.first] += index_entry.second.size();
    }
    ci.messageIndexLength = out_.size() - index_start;
    ci.compression = job.compression;
    ci.compressedSize = job.compressed.size();
    ci.uncompressedSize = job.uncompressed_size;
    chunk_indexes_.push_back(std::move(ci));

    message_count_ += job.message_count;
    if (job.message_count > 0) {
      min_time_ = std::min(min_time_, job.start_time);
      max_time_ = std::max(max_time_, job.end_time);
    }
  }

  // Same layout as the pass-through engine's summary: repeated schemas and
  // channels, chunk indexes, statistics, summary offsets, footer with the
  // summary CRC, trailing magic.
  void write_summary_section()
  {
    const std::uint64_t summary_start = out_.size();
    out_.crcEnabled = true;
    out_.resetCrc();

    struct Group
    {
      mcap::OpCode opcode;
      std::uint64_t start;
      std::uint64_t length;
    };
    std::vector<Group> groups;

    if (!schemas_.empty()) {
      const std::uint64_t start = out_.size();
      for (const auto & schema : schemas_) {
        mcap::McapWriter::write(out_, schema);
      }
      groups.push_back({mcap::OpCode::Schema, start, out_.size() - start});
    }
    if (!channels_.empty()) {
      const std::uint64_t start = out_.size();
      for (const auto & channel : channels_) {
        mcap::McapWriter::write(out_, channel);
      }
      groups.push_back({mcap::OpCode::Channel, start, out_.size() - start});
    }
    if (!chunk_indexes_.empty()) {
      const std::uint64_t start = out_.size();
      for (const auto & ci : chunk_indexes_) {
        mcap::McapWriter::write(out_, ci);
      }
      groups.push_back({mcap::OpCode::ChunkIndex, start, out_.size() - start});
    }
    {
      const std::uint64_t start = out_.size();
      mcap::Statistics stats;
      stats.messageCount = message_count_;
      stats.schemaCount = static_cast<std::uint16_t>(schemas_.size());
      stats.channelCount = static_cast<std::uint32_t>(channels_.size());
      stats.attachmentCount = 0;
      stats.metadataCount = 0;
      stats.chunkCount = static_cast<std::uint32_t>(chunk_indexes_.size());
      stats.messageStartTime = message_count_ > 0 ? min_time_ : 0;
      stats.messageEndTime = message_count_ > 0 ? max_time_ : 0;
      for (const auto & channel_count : channel_counts_) {
        stats.channelMessageCounts[channel_count.first] = channel_count.second;
      }
      mcap::McapWriter::write(out_, stats);
      groups.push_back({mcap::OpCode::Statistics, start, out_.size() - start});
    }

    const std::uint64_t summary_offset_start = out_.size();
    for (const auto & group : groups) {
      mcap::McapWriter::write(out_, mcap::SummaryOffset{group.opcode, group.start, group.length});
    }
    mcap::McapWriter::write(
      out_, mcap::Footer{summary_start, summary_offset_start}, /*crcEnabled=*/true);
    mcap::McapWriter::writeMagic(out_);
  }

  const std::string compression_;
  const std::uint64_t chunk_size_;
  const std::size_t max_inflight_;

  mcap::FileWriter out_;
  std::optional<WritebackWindow> writeback_window_;
  bool closed_ = false;

  // Caller-thread staging state.
  std::vector<Segment> staging_segments_;
  std::uint64_t staging_size_ = 0;  // record bytes staged so far
  std::map<mcap::ChannelId, std::vector<std::pair<mcap::Timestamp, mcap::ByteOffset>>>
    staging_index_;
  mcap::Timestamp staging_times_min_ = mcap::MaxTime;
  mcap::Timestamp staging_times_max_ = 0;
  std::uint64_t staging_count_ = 0;
  std::uint64_t next_seq_ = 0;

  // Declared records, kept for the summary section. Written to the data
  // section as records jobs, so declare order is preserved in the file.
  std::vector<mcap::Schema> schemas_;
  std::vector<mcap::Channel> channels_;

  // Sequencer-accumulated summary state.
  std::vector<mcap::ChunkIndex> chunk_indexes_;
  std::map<mcap::ChannelId, std::uint64_t> channel_counts_;
  std::uint64_t message_count_ = 0;
  mcap::Timestamp min_time_ = mcap::MaxTime;
  mcap::Timestamp max_time_ = 0;

  // Shared queue state.
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::unique_ptr<Job>> jobs_;
  std::vector<std::vector<std::byte>> buffer_pool_;
  bool stop_ = false;
  std::string error_;

  std::deque<Encoder> encoders_;
  std::vector<std::jthread> workers_;
  std::jthread sequencer_;
};

ParallelChunkMcapWriter::ParallelChunkMcapWriter(
  const std::filesystem::path & path, std::string compression, mcap::CompressionLevel level,
  std::uint64_t chunk_size, int num_threads)
: impl_(std::make_unique<Impl>(path, std::move(compression), level, chunk_size, num_threads))
{
}

ParallelChunkMcapWriter::~ParallelChunkMcapWriter() = default;

void ParallelChunkMcapWriter::write_schema(const mcap::Schema & schema)
{
  impl_->write_schema(schema);
}

void ParallelChunkMcapWriter::write_channel(const mcap::Channel & channel)
{
  impl_->write_channel(channel);
}

void ParallelChunkMcapWriter::write_message(const mcap::Message & message)
{
  impl_->write_message(message);
}

void ParallelChunkMcapWriter::write_message_owned(
  mcap::ChannelId channel, mcap::Timestamp log_time, std::span<const std::byte> payload,
  std::shared_ptr<const void> owner)
{
  impl_->write_message_owned(channel, log_time, payload, std::move(owner));
}

void ParallelChunkMcapWriter::close()
{
  impl_->close();
}

}  // namespace bagwiz::io::detail
