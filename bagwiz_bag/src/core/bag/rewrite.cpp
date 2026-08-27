// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/rewrite.hpp"

#include "bagwiz/core/bag/bag_inplace.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace bagwiz::core
{

namespace
{

// Stamp the command's compression choices onto the CreateOptions the branch
// composed. Shared by both branches so an override behaves identically
// whether it lands on an -o output or on the in-place tmp; an empty string
// leaves that knob at the storage default.
void apply_compression_overrides(io::CreateOptions & copts, const BagRewriteOptions & options)
{
  if (!options.mcap_compression.empty()) {
    copts.mcap_compression = options.mcap_compression;
  }
  if (!options.mcap_compression_level.empty()) {
    copts.mcap_compression_level = options.mcap_compression_level;
  }
  if (!options.sqlite3_compression_mode.empty()) {
    copts.sqlite3_compression_mode = options.sqlite3_compression_mode;
  }
  if (!options.sqlite3_compression_format.empty()) {
    copts.sqlite3_compression_format = options.sqlite3_compression_format;
  }
  if (!options.sqlite3_compression_level.empty()) {
    copts.sqlite3_compression_level = options.sqlite3_compression_level;
  }
}

}  // namespace

int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePassWithTarget & pass)
{
  // -o mode: write a new bag whose storage follows the output path (its
  // extension picks a single-file backend; a directory inherits the input's
  // backend when inherit_output_format is set) and leave <input> untouched.
  if (output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*output_path, overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(options.logger, "%s", r.error.c_str());
      return 1;
    }
    const auto output = *output_path;
    io::CreateOptions copts;
    if (options.output_format != io::Format::Auto) {
      // The command already resolved the backend; only the layout is left to
      // the output path's extension.
      copts.format = options.output_format;
      copts.layout = io::Layout::Auto;
    } else if (options.inherit_output_format) {
      copts = io::create_options_inheriting_format(input_path, output);
    } else {
      copts.format = io::Format::Auto;
      copts.layout = io::Layout::Auto;
    }
    apply_compression_overrides(copts, options);
    const io::WriterFactory make_writer = [output, copts]() {
      return io::open_write(output, copts);
    };
    return pass(make_writer, RewriteTarget{output, copts});
  }

  // In-place mode: rewrite <input> atomically via a staged sibling copy,
  // preserving its storage format and layout. The staged path keeps the
  // bag's own name, but a directory bag's name carries no extension for
  // Format::Auto to resolve from (it would fall through to the
  // Directory + Mcap default), so pin both explicitly.
  auto inplace_copts = io::create_options_preserving_storage(input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(options.logger, options.format_unknown_error, input_path.string().c_str());
    return 1;
  }
  apply_compression_overrides(inplace_copts, options);

  // The pass reports command-level failures via its return value rather than
  // throwing, so capture the status and translate a non-zero exit into a
  // runtime_error to make write_bag_inplace abort the swap (leaving <input>
  // untouched).
  int pass_status = 0;
  try {
    core::write_bag_inplace(input_path, [&](const std::filesystem::path & tmp) {
      const io::WriterFactory make_writer = [&tmp, &inplace_copts]() {
        return io::open_write(tmp, inplace_copts);
      };
      pass_status = pass(make_writer, RewriteTarget{tmp, inplace_copts});
      if (pass_status != 0) {
        throw std::runtime_error(options.pass_failed_error);
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;  // the pass already logged the specific error
    }
    BAGWIZ_LOG_ERROR(options.logger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

int run_bag_rewrite(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite,
  const BagRewriteOptions & options, const BagRewritePass & pass)
{
  return run_bag_rewrite(
    input_path, output_path, overwrite, options,
    [&pass](const io::WriterFactory & make_writer, const RewriteTarget &) {
      return pass(make_writer);
    });
}

}  // namespace bagwiz::core
