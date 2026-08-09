// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/page_cache.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace
{

// RAII guard for one environment variable: applies the requested state (a
// value to set, or std::nullopt to unset) for the scope and restores the
// previous state on destruction, so a test neither depends on nor leaks the
// ambient environment.
class EnvVarGuard
{
public:
  EnvVarGuard(const char * name, const std::optional<std::string> & value) : name_(name)
  {
    if (const char * previous = ::getenv(name); previous != nullptr) {
      previous_value_ = previous;
    }
    if (value.has_value()) {
      ::setenv(name, value->c_str(), 1);
    } else {
      ::unsetenv(name);
    }
  }

  EnvVarGuard(const EnvVarGuard &) = delete;
  EnvVarGuard & operator=(const EnvVarGuard &) = delete;
  EnvVarGuard(EnvVarGuard &&) = delete;
  EnvVarGuard & operator=(EnvVarGuard &&) = delete;

  ~EnvVarGuard()
  {
    if (previous_value_.has_value()) {
      ::setenv(name_.c_str(), previous_value_->c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> previous_value_;
};

#ifdef __linux__
constexpr std::size_t kFileBytes = 8ull * 1024 * 1024;  // 8 MiB

// Writes `bytes` of content and fsyncs, so the file's pages are clean — a
// DONTNEED drop only evicts clean pages, and leaving them dirty would make the
// residency assertions below racy.
void write_file(const std::filesystem::path & path, std::size_t bytes)
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  ASSERT_GE(fd, 0) << std::strerror(errno);
  std::vector<char> chunk(1u << 20, '\xAB');
  std::size_t left = bytes;
  while (left > 0) {
    const std::size_t want = std::min(left, chunk.size());
    ASSERT_EQ(::write(fd, chunk.data(), want), static_cast<ssize_t>(want));
    left -= want;
  }
  ASSERT_EQ(::fsync(fd), 0);
  ASSERT_EQ(::close(fd), 0);
}

// Reads the whole file, pulling its pages into the page cache.
void populate_cache(const std::filesystem::path & path)
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  ASSERT_GE(fd, 0) << std::strerror(errno);
  std::vector<char> buf(1u << 20);
  ssize_t got;
  while ((got = ::read(fd, buf.data(), buf.size())) > 0) {
  }
  ASSERT_EQ(got, 0) << std::strerror(errno);
  ASSERT_EQ(::close(fd), 0);
}

// Fraction of the file's pages currently resident in the page cache, measured
// with mincore(2) over a private mapping (mapping alone does not populate).
// Returns a negative value on any lookup failure.
double resident_fraction(const std::filesystem::path & path)
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    return -1.0;
  }
  struct stat st;
  if (::fstat(fd, &st) != 0 || st.st_size <= 0) {
    ::close(fd);
    return -1.0;
  }
  const auto size = static_cast<std::size_t>(st.st_size);
  void * map = ::mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0);
  ::close(fd);
  if (map == MAP_FAILED) {
    return -1.0;
  }
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const std::size_t pages = (size + page - 1) / page;
  std::vector<unsigned char> vec(pages, 0);
  double result = -1.0;
  if (::mincore(map, size, vec.data()) == 0) {
    std::size_t resident = 0;
    for (const auto v : vec) {
      resident += v & 1u;
    }
    result = static_cast<double>(resident) / static_cast<double>(pages);
  }
  ::munmap(map, size);
  return result;
}
#endif  // __linux__

class PageCacheTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Deterministic knob state regardless of the ambient environment.
    env_guard_.emplace("BAGWIZ_PAGE_CACHE_DROP", "1");
    tmp_dir_ = std::filesystem::temp_directory_path() /
               (std::string("bagwiz_page_cache_test_") +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    {
      // Drain the registry without dropping, so no entry registered by a test
      // leaks into the next one (or into the atexit pass at process exit).
      EnvVarGuard off("BAGWIZ_PAGE_CACHE_DROP", "0");
      bagwiz::io::drop_registered_file_caches();
    }
    env_guard_.reset();
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::optional<EnvVarGuard> env_guard_;
  std::filesystem::path tmp_dir_;
};

TEST_F(PageCacheTest, ReadFileDropEvictsPages)
{
#ifdef __linux__
  const auto file = tmp_dir_ / "input.mcap";
  write_file(file, kFileBytes);
  populate_cache(file);
  ASSERT_GT(resident_fraction(file), 0.9);

  bagwiz::io::register_read_file(file);
  bagwiz::io::drop_registered_file_caches();

  const double after = resident_fraction(file);
  ASSERT_GE(after, 0.0);
  EXPECT_LT(after, 0.05);
#else
  GTEST_SKIP() << "page-cache drop is implemented for Linux only";
#endif
}

TEST_F(PageCacheTest, WrittenFileDropEvictsPages)
{
#ifdef __linux__
  const auto file = tmp_dir_ / "output.mcap";
  write_file(file, kFileBytes);
  populate_cache(file);
  ASSERT_GT(resident_fraction(file), 0.9);

  bagwiz::io::register_written_file(file);
  bagwiz::io::drop_registered_file_caches();

  const double after = resident_fraction(file);
  ASSERT_GE(after, 0.0);
  EXPECT_LT(after, 0.05);
#else
  GTEST_SKIP() << "page-cache drop is implemented for Linux only";
#endif
}

TEST_F(PageCacheTest, DirectoryRegistrationDropsShards)
{
#ifdef __linux__
  const auto bag_dir = tmp_dir_ / "bag";
  std::filesystem::create_directories(bag_dir);
  const auto shard0 = bag_dir / "bag_0.mcap";
  const auto shard1 = bag_dir / "bag_1.mcap";
  write_file(shard0, kFileBytes / 2);
  write_file(shard1, kFileBytes / 2);
  populate_cache(shard0);
  populate_cache(shard1);
  ASSERT_GT(resident_fraction(shard0), 0.9);
  ASSERT_GT(resident_fraction(shard1), 0.9);

  bagwiz::io::register_read_file(bag_dir);
  bagwiz::io::drop_registered_file_caches();

  EXPECT_LT(resident_fraction(shard0), 0.05);
  EXPECT_LT(resident_fraction(shard1), 0.05);
#else
  GTEST_SKIP() << "page-cache drop is implemented for Linux only";
#endif
}

TEST_F(PageCacheTest, DisabledEnvKeepsPages)
{
#ifdef __linux__
  EnvVarGuard off("BAGWIZ_PAGE_CACHE_DROP", "0");
  const auto file = tmp_dir_ / "input.mcap";
  write_file(file, kFileBytes);
  populate_cache(file);
  ASSERT_GT(resident_fraction(file), 0.9);

  bagwiz::io::register_read_file(file);
  bagwiz::io::drop_registered_file_caches();

  EXPECT_GT(resident_fraction(file), 0.9);
#else
  GTEST_SKIP() << "page-cache drop is implemented for Linux only";
#endif
}

TEST_F(PageCacheTest, MissingPathIsIgnored)
{
  bagwiz::io::register_read_file(tmp_dir_ / "gone.mcap");
  bagwiz::io::register_written_file(tmp_dir_ / "gone.mcap");
  bagwiz::io::drop_registered_file_caches();
  SUCCEED();
}

TEST_F(PageCacheTest, UnregisteredWrittenFileKeepsPages)
{
#ifdef __linux__
  const auto file = tmp_dir_ / "output.mcap";
  write_file(file, kFileBytes);
  populate_cache(file);
  ASSERT_GT(resident_fraction(file), 0.9);

  // A writer that manages its own drop (detail::WritebackWindow) unregisters
  // when finished; the exit-time pass must then leave the file alone.
  bagwiz::io::register_written_file(file);
  bagwiz::io::unregister_written_file(file);
  bagwiz::io::drop_registered_file_caches();

  EXPECT_GT(resident_fraction(file), 0.9);
#else
  GTEST_SKIP() << "page-cache drop is implemented for Linux only";
#endif
}

}  // namespace
