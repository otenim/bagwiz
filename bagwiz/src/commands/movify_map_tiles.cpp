// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_tiles.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <curl/curl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr std::string_view kFileScheme = "file://";
constexpr std::string_view kHttpScheme = "http://";
constexpr std::string_view kHttpsScheme = "https://";
constexpr std::string_view kOsmHost = "openstreetmap.org";
constexpr std::string_view kOsmAttribution = "(c) OpenStreetMap contributors";
constexpr long kMapTileConnectTimeoutS = 10;  // NOLINT(runtime/int) libcurl takes a long
constexpr long kMapTileMaxRedirects = 5;      // NOLINT(runtime/int) libcurl takes a long

std::string replace_all(std::string text, std::string_view from, const std::string & to)
{
  for (auto pos = text.find(from); pos != std::string::npos;
       pos = text.find(from, pos + to.size())) {
    text.replace(pos, from.size(), to);
  }
  return text;
}

std::optional<std::string> env_var(const char * name)
{
  const char * const value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::string{value};
}

// The template's authority (host[:port]), for the cache's per-server
// subdirectory and the attribution: "" for a file template.
std::string host_of(const std::string & url_template)
{
  std::string rest;
  if (url_template.starts_with(kHttpsScheme)) {
    rest = url_template.substr(kHttpsScheme.size());
  } else if (url_template.starts_with(kHttpScheme)) {
    rest = url_template.substr(kHttpScheme.size());
  } else {
    return "";
  }
  return rest.substr(0, rest.find('/'));
}

// The host with every character a path should not carry replaced.
std::string cache_subdir_of(const std::string & host)
{
  std::string out = host;
  for (char & c : out) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_')) {
      c = '_';
    }
  }
  return out;
}

// --- transport ---------------------------------------------------------

struct FetchResult
{
  std::vector<unsigned char> bytes;
  std::string error;  // "" on success
};

FetchResult read_file(const std::filesystem::path & path)
{
  FetchResult out;
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    out.error = "cannot read '" + path.string() + "'";
    return out;
  }
  out.bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  if (out.bytes.empty()) {
    out.error = "'" + path.string() + "' is empty";
  }
  return out;
}

// libcurl's global state is initialized once for the process, on first use;
// it is torn down with the process (a function-local static's destructor).
void ensure_curl_initialized()
{
  struct CurlGlobal
  {
    CurlGlobal() { curl_global_init(CURL_GLOBAL_DEFAULT); }
    ~CurlGlobal() { curl_global_cleanup(); }
    CurlGlobal(const CurlGlobal &) = delete;
    CurlGlobal & operator=(const CurlGlobal &) = delete;
  };
  static const CurlGlobal global;
}

std::size_t append_bytes(char * data, std::size_t size, std::size_t count, void * user)
{
  auto * bytes = static_cast<std::vector<unsigned char> *>(user);
  const std::size_t total = size * count;
  bytes->insert(bytes->end(), data, data + total);
  return total;
}

// One easy handle per thread, kept across requests so libcurl reuses the
// connection to the tile server (a handle per request would open a fresh
// TCP + TLS session for every tile). The prefetch joins its workers before
// returning, and the main thread's handle goes before the process's static
// libcurl state does, so every handle is cleaned up ahead of
// curl_global_cleanup().
CURL * thread_curl_handle()
{
  ensure_curl_initialized();
  thread_local const std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> handle(
    curl_easy_init(), &curl_easy_cleanup);
  return handle.get();
}

FetchResult http_get(
  const std::string & url, const std::string & user_agent,
  long timeout_s)  // NOLINT(runtime/int) libcurl takes a long
{
  FetchResult out;
  CURL * const curl_handle = thread_curl_handle();
  if (curl_handle == nullptr) {
    out.error = "libcurl could not be initialized";
    return out;
  }
  const std::unique_ptr<CURL, decltype(&curl_easy_reset)> curl(curl_handle, &curl_easy_reset);
  curl_easy_reset(curl.get());  // only the connection cache survives a reset
  curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_USERAGENT, user_agent.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_MAXREDIRS, kMapTileMaxRedirects);
  curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT, kMapTileConnectTimeoutS);
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, timeout_s);
  curl_easy_setopt(curl.get(), CURLOPT_NOSIGNAL, 1L);  // threads: no SIGALRM timeouts
  curl_easy_setopt(curl.get(), CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_ACCEPT_ENCODING, "");
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, &append_bytes);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &out.bytes);
  const CURLcode code = curl_easy_perform(curl.get());
  if (code != CURLE_OK) {
    long status = 0;  // NOLINT(runtime/int) libcurl takes a long
    curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
    out.error = std::string{curl_easy_strerror(code)};
    if (status != 0) {
      out.error += " (HTTP " + std::to_string(status) + ")";
    }
    out.bytes.clear();
  } else if (out.bytes.empty()) {
    out.error = "empty response";
  }
  return out;
}

// Write `bytes` to `path` through a sibling temp file, so a reader never
// sees a half-written tile. The temp name is unique to this process and
// write: the cache is shared by every bagwiz process, and two runs fetching
// the same tile at once must not truncate each other's half-written file
// (the last rename wins, whole). Best effort: a cache that cannot be
// written is only a cache.
void store_cached(const std::filesystem::path & path, const std::vector<unsigned char> & bytes)
{
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    return;
  }
  static std::atomic<unsigned> serial{0};
  const auto tmp = path.string() + ".part-" + std::to_string(::getpid()) + "-" +
                   std::to_string(serial.fetch_add(1));
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) {
      return;
    }
    out.write(
      reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!out) {
      std::filesystem::remove(tmp, ec);
      return;
    }
  }
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    std::filesystem::remove(tmp, ec);
  }
}

std::optional<cv::Mat> decode_tile(const std::vector<unsigned char> & bytes, std::string & error)
{
  cv::Mat image = cv::imdecode(bytes, cv::IMREAD_COLOR);
  if (image.empty()) {
    error = "not a decodable image";
    return std::nullopt;
  }
  if (image.rows != kMapTileSizePx || image.cols != kMapTileSizePx) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(kMapTileSizePx, kMapTileSizePx), 0, 0, cv::INTER_AREA);
    image = resized;
  }
  return image;
}
}  // namespace

MercatorPixel mercator_pixel(double latitude_deg, double longitude_deg, int zoom)
{
  const double world_px = static_cast<double>(kMapTileSizePx) * std::ldexp(1.0, zoom);
  const double lat = latitude_deg * std::numbers::pi / 180.0;
  MercatorPixel out;
  out.x = (longitude_deg + 180.0) / 360.0 * world_px;
  out.y = (1.0 - std::asinh(std::tan(lat)) / std::numbers::pi) / 2.0 * world_px;
  return out;
}

double mercator_meters_per_pixel(double latitude_deg, int zoom)
{
  return kMercatorEquatorMetersPerPixel * std::cos(latitude_deg * std::numbers::pi / 180.0) /
         std::ldexp(1.0, zoom);
}

int map_tile_zoom(double latitude_deg, double px_per_m, int max_zoom)
{
  // Tiles at zoom z span mercator_meters_per_pixel(lat, z) per pixel; the
  // panel wants at most 1 / px_per_m. Solving for z and rounding up gives
  // the coarsest zoom that is still fine enough.
  const double scale = mercator_meters_per_pixel(latitude_deg, 0) * px_per_m;
  if (!(scale > 1.0)) {
    return 0;
  }
  const double zoom = std::ceil(std::log2(scale));
  return static_cast<int>(std::clamp(zoom, 0.0, static_cast<double>(max_zoom)));
}

std::string expand_map_tile_template(const std::string & url_template, const MapTileKey & key)
{
  std::string url = replace_all(url_template, "{z}", std::to_string(key.zoom));
  url = replace_all(url, "{x}", std::to_string(key.x));
  return replace_all(url, "{y}", std::to_string(key.y));
}

std::string validate_map_tile_template(const std::string & url_template)
{
  if (url_template == kMapTilesNone) {
    return "";
  }
  if (
    !url_template.starts_with(kHttpScheme) && !url_template.starts_with(kHttpsScheme) &&
    !url_template.starts_with(kFileScheme)) {
    return "--map-tiles '" + url_template +
           "' is not a http://, https:// or file:// URL template (or 'none').";
  }
  for (const std::string_view placeholder : {"{z}", "{x}", "{y}"}) {
    if (url_template.find(placeholder) == std::string::npos) {
      return "--map-tiles '" + url_template + "' lacks the " + std::string{placeholder} +
             " placeholder (a tile URL template needs {z}, {x} and {y}).";
    }
  }
  return "";
}

std::string map_tile_attribution(const std::string & url_template)
{
  const std::string host = host_of(url_template);
  const std::string bare = host.substr(0, host.find(':'));
  if (bare == kOsmHost || bare.ends_with(std::string{"."} + std::string{kOsmHost})) {
    return std::string{kOsmAttribution};
  }
  return "";
}

std::optional<std::filesystem::path> default_map_tile_cache_dir()
{
  if (const auto xdg = env_var("XDG_CACHE_HOME"); xdg.has_value()) {
    return std::filesystem::path(*xdg) / "bagwiz" / "tiles";
  }
  if (const auto home = env_var("HOME"); home.has_value()) {
    return std::filesystem::path(*home) / ".cache" / "bagwiz" / "tiles";
  }
  return std::nullopt;
}

HttpMapTileSource::HttpMapTileSource(Options options) : options_(std::move(options))
{
}

std::optional<std::filesystem::path> HttpMapTileSource::cache_path_of(const MapTileKey & key) const
{
  if (!options_.cache_dir.has_value() || options_.url_template.starts_with(kFileScheme)) {
    return std::nullopt;
  }
  return *options_.cache_dir / cache_subdir_of(host_of(options_.url_template)) /
         std::to_string(key.zoom) / std::to_string(key.x) / (std::to_string(key.y) + ".png");
}

void HttpMapTileSource::note_error(const std::string & error)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  last_error_ = error;
}

std::string HttpMapTileSource::last_error() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

std::optional<cv::Mat> HttpMapTileSource::tile(const MapTileKey & key)
{
  const std::string url = expand_map_tile_template(options_.url_template, key);
  const auto cache_path = cache_path_of(key);
  std::string decode_error;
  // A cached tile that no longer decodes (a run cut short, a disk hiccup)
  // is dropped and fetched afresh rather than reported missing forever.
  if (cache_path.has_value() && std::filesystem::exists(*cache_path)) {
    const FetchResult cached = read_file(*cache_path);
    if (cached.error.empty()) {
      if (auto image = decode_tile(cached.bytes, decode_error); image.has_value()) {
        return image;
      }
    }
    std::error_code ec;
    std::filesystem::remove(*cache_path, ec);
  }
  const FetchResult fetched = url.starts_with(kFileScheme)
                                ? read_file(url.substr(kFileScheme.size()))
                                : http_get(url, options_.user_agent, options_.timeout_s);
  if (!fetched.error.empty()) {
    note_error(url + ": " + fetched.error);
    return std::nullopt;
  }
  auto image = decode_tile(fetched.bytes, decode_error);
  if (!image.has_value()) {
    note_error(url + ": " + decode_error);
    return std::nullopt;
  }
  if (cache_path.has_value()) {
    store_cached(*cache_path, fetched.bytes);
  }
  return image;
}

}  // namespace bagwiz::commands
