// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/message_formatter.hpp"

#include "bagwiz/core/introspection_loader.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

// Hand-crafted CDR-LE payload for a std_msgs/msg/String with data="hello".
// Layout:
//   encapsulation header  : 00 01 00 00   (CDR_LE)
//   string length (uint32): 06 00 00 00   (5 chars + NUL)
//   string bytes          : 68 65 6C 6C 6F 00   ("hello\0")
std::vector<std::byte> make_string_payload(const std::string & value)
{
  std::vector<std::byte> out;
  const auto push = [&out](std::uint8_t b) { out.push_back(std::byte{b}); };
  push(0x00);
  push(0x01);
  push(0x00);
  push(0x00);
  const std::uint32_t length = static_cast<std::uint32_t>(value.size()) + 1;  // + NUL
  push(static_cast<std::uint8_t>(length & 0xFF));
  push(static_cast<std::uint8_t>((length >> 8) & 0xFF));
  push(static_cast<std::uint8_t>((length >> 16) & 0xFF));
  push(static_cast<std::uint8_t>((length >> 24) & 0xFF));
  for (const char c : value) {
    push(static_cast<std::uint8_t>(c));
  }
  push(0x00);
  return out;
}

// Exercises the full pipeline: dlopen both typesupports, rmw_deserialize
// the CDR bytes into a populated in-memory struct, walk the struct via
// introspection, emit YAML. std_msgs is guaranteed to be available in any
// ROS 2 installation so this remains a fast, self-contained smoke test.
TEST(MessageFormatter, DecodesStdMsgsString)
{
  const auto introspection = bagwiz::core::load_introspection("std_msgs/msg/String");
  ASSERT_TRUE(introspection.ok()) << "load error: " << introspection.error
                                  << " (lib: " << introspection.library_name << ")";

  const auto payload = make_string_payload("hello");
  const auto result = bagwiz::core::format_message(introspection, payload);

  ASSERT_TRUE(result.ok()) << "format error: " << result.error;
  EXPECT_NE(result.text.find("data: 'hello'"), std::string::npos) << "got:\n" << result.text;
}

TEST(MessageFormatter, ReportsErrorOnTruncatedPayload)
{
  const auto introspection = bagwiz::core::load_introspection("std_msgs/msg/String");
  ASSERT_TRUE(introspection.ok());

  // Only the encapsulation header: no string body, should fail to
  // deserialize rather than crash.
  std::vector<std::byte> too_short = {
    std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
  const auto result = bagwiz::core::format_message(introspection, too_short);
  EXPECT_FALSE(result.ok());
  EXPECT_TRUE(result.text.empty());
}

TEST(MessageFormatter, RejectsUnknownType)
{
  const auto introspection =
    bagwiz::core::load_introspection("definitely_not_a_real_pkg/msg/Ghost");
  EXPECT_FALSE(introspection.ok());
  EXPECT_FALSE(introspection.error.empty());
}

}  // namespace
