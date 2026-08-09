// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/writeback_window.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

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

constexpr std::uint64_t kInterval = 1ull << 20;  // 1 MiB

#ifdef __linux__
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

class WritebackWindowTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               (std::string("bagwiz_writeback_window_test_") +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(WritebackWindowTest, FinishKeepsWrittenPages)
{
#ifdef __linux__
  const auto file = tmp_dir_ / "out.mcap";
  const int fd = ::open(file.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  ASSERT_GE(fd, 0) << std::strerror(errno);

  // The window paces writeback but never evicts: finish() flushes the bounded
  // remainder and leaves the pages in the cache.
  bagwiz::io::detail::WritebackWindow window(file, kInterval);
  std::vector<char> chunk(kInterval / 4, '\xCD');  // 256 KiB per write
  std::uint64_t written = 0;
  for (int i = 0; i < 16; ++i) {  // 4 MiB total
    ASSERT_EQ(::write(fd, chunk.data(), chunk.size()), static_cast<ssize_t>(chunk.size()));
    written += chunk.size();
    window.note_offset(written);
  }
  ASSERT_EQ(::close(fd), 0);

  window.finish();  // waits for the bounded remainder

  EXPECT_GT(resident_fraction(file), 0.9);
#else
  GTEST_SKIP() << "the writeback window is implemented for Linux only";
#endif
}

TEST_F(WritebackWindowTest, ZeroIntervalIsNoOp)
{
#ifdef __linux__
  const auto file = tmp_dir_ / "out.mcap";
  const int fd = ::open(file.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  ASSERT_GE(fd, 0) << std::strerror(errno);

  bagwiz::io::detail::WritebackWindow window(file, 0);
  std::vector<char> chunk(kInterval / 4, '\xCD');
  std::uint64_t written = 0;
  for (int i = 0; i < 8; ++i) {  // 2 MiB total, left dirty on purpose
    ASSERT_EQ(::write(fd, chunk.data(), chunk.size()), static_cast<ssize_t>(chunk.size()));
    written += chunk.size();
    window.note_offset(written);
  }
  ASSERT_EQ(::close(fd), 0);
  window.finish();

  // Nothing was synced: the just-written pages stay resident.
  EXPECT_GT(resident_fraction(file), 0.9);
#else
  GTEST_SKIP() << "the writeback window is implemented for Linux only";
#endif
}

TEST_F(WritebackWindowTest, MissingFileIsNoOp)
{
  bagwiz::io::detail::WritebackWindow window(tmp_dir_ / "gone" / "out.mcap", kInterval);
  window.note_offset(16 * kInterval);
  window.finish();
  SUCCEED();
}

TEST_F(WritebackWindowTest, FinishIsIdempotent)
{
  bagwiz::io::detail::WritebackWindow window(tmp_dir_ / "gone" / "out.mcap", kInterval);
  window.finish();
  window.finish();
  SUCCEED();
}

TEST(WritebackIntervalTest, EnvResolution)
{
  constexpr const char * kName = "BAGWIZ_WRITEBACK_INTERVAL_BYTES";
  constexpr std::uint64_t kDefault = 256ull * 1024 * 1024;
  {
    EnvVarGuard unset(kName, std::nullopt);
    EXPECT_EQ(bagwiz::io::detail::resolve_writeback_interval_bytes("bagwiz.test"), kDefault);
  }
  {
    EnvVarGuard zero(kName, "0");
    EXPECT_EQ(bagwiz::io::detail::resolve_writeback_interval_bytes("bagwiz.test"), 0u);
  }
  {
    EnvVarGuard custom(kName, "1048576");
    EXPECT_EQ(bagwiz::io::detail::resolve_writeback_interval_bytes("bagwiz.test"), 1u << 20);
  }
  {
    EnvVarGuard garbage(kName, "not-a-number");
    EXPECT_EQ(bagwiz::io::detail::resolve_writeback_interval_bytes("bagwiz.test"), kDefault);
  }
}

}  // namespace
