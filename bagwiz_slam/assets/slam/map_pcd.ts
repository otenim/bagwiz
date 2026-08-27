// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// PCD reader for the map viewer, split out of map_viewer.ts.
//
// three.js's PCDLoader decodes the *whole* file to one JavaScript string before
// it looks for the header, and it accumulates every field into plain JS arrays
// before copying them into typed attributes. Both break on the clouds
// `bagwiz map slam` writes: a 43 M-point map.pcd is 860 MB, past V8's ~512 M
// character cap on a single string, so TextDecoder hands back "" and PCDLoader
// fails on the empty header with "Cannot read properties of null (reading '1')"
// -- and the intermediate arrays would want several GB even if it did not.
//
// So the format `map slam` writes -- DATA binary -- is read here instead: the
// header comes from a bounded prefix, and the points go straight into
// preallocated typed arrays. `ascii` and `binary_compressed` (clouds from other
// tools) still go through PCDLoader, with an explicit error when such a file is
// past the size it can decode.

import { Color, SRGBColorSpace } from "three";
import { PCDLoader } from "three/addons/loaders/PCDLoader.js";

// One parsed cloud. Every array is owned by the caller (nothing else holds a
// reference), so the viewer can install them as attributes without copying.
export interface PcdCloud {
  count: number;
  position: Float32Array; // count*3, xyz in the cloud's own frame
  intensity: Float32Array | null; // count, null when the field is absent
  color: Float32Array | null; // count*3 in three's working space, null when absent
}

// A PCD header is ~10 short ASCII lines, so its terminating DATA line is found
// in a bounded prefix rather than by scanning the (huge) point data behind it.
const HEADER_SCAN_BYTES = 64 * 1024;

// V8 caps a string at 2^29-24 characters. A decode never yields more characters
// than input bytes, so a buffer at or under the cap is always decodable whole.
const MAX_DECODABLE_BYTES = 0x1fffffe8;

// PCD binary payloads are little-endian (PCDLoader assumes the same).
const LITTLE_ENDIAN = true;

// Reads one numeric field out of a point row.
type FieldReader = (view: DataView, byteOffset: number) => number;

interface PcdField {
  read: FieldReader;
  offset: number; // byte offset within a point row
}

interface PcdHeader {
  data: string; // ascii | binary | binary_compressed
  headerLen: number; // bytes before the first point
  fields: string[];
  size: number[]; // bytes per element, per field
  type: string[]; // I | U | F, per field
  points: number;
  offset: Record<string, number>; // byte offset within a row, per field name
  rowSize: number;
}

// PCDLoader feeds the rgb bytes through Color.setRGB(..., SRGBColorSpace) --
// sRGB into three's linear working space. Match it exactly through the same
// call rather than re-deriving the transfer function; a byte has only 256
// values, so the whole mapping precomputes.
function buildSrgbByteToLinear(): Float32Array {
  const lut = new Float32Array(256);
  const color = new Color();
  for (let value = 0; value < lut.length; value += 1) {
    const unit = value / 255;
    color.setRGB(unit, unit, unit, SRGBColorSpace);
    lut[value] = color.r;
  }
  return lut;
}

const SRGB_BYTE_TO_LINEAR = buildSrgbByteToLinear();

function fieldReader(type: string, size: number): FieldReader | null {
  const key = `${type.toUpperCase()}${size}`;
  switch (key) {
    case "F4":
      return (view, at) => view.getFloat32(at, LITTLE_ENDIAN);
    case "F8":
      return (view, at) => view.getFloat64(at, LITTLE_ENDIAN);
    case "I1":
      return (view, at) => view.getInt8(at);
    case "I2":
      return (view, at) => view.getInt16(at, LITTLE_ENDIAN);
    case "I4":
      return (view, at) => view.getInt32(at, LITTLE_ENDIAN);
    case "U1":
      return (view, at) => view.getUint8(at);
    case "U2":
      return (view, at) => view.getUint16(at, LITTLE_ENDIAN);
    case "U4":
      return (view, at) => view.getUint32(at, LITTLE_ENDIAN);
    default:
      return null;
  }
}

// Value of a `KEYWORD rest of line` header entry, or null when absent.
function keywordValue(header: string, keyword: string): string | null {
  const match = new RegExp(`^[ \\t]*${keyword}[ \\t]+(.*)$`, "im").exec(header);
  return match ? match[1].trim() : null;
}

function words(value: string | null): string[] {
  return value ? value.split(/\s+/) : [];
}

// Parse `value` as a count of points/elements; anything that is not a positive
// whole number is a malformed header rather than something to guess around.
function positiveInt(value: string | null, what: string): number {
  const parsed = Number(value);
  if (!Number.isSafeInteger(parsed) || parsed <= 0) {
    throw new Error(`PCD header has an invalid ${what}: ${String(value)}`);
  }
  return parsed;
}

function parseHeader(buffer: ArrayBuffer): PcdHeader {
  const scanLen = Math.min(buffer.byteLength, HEADER_SCAN_BYTES);
  // latin1 maps every byte to exactly one character, so offsets into `prefix`
  // are byte offsets into `buffer` (a utf-8 decode would not guarantee that).
  const prefix = new TextDecoder("latin1").decode(new Uint8Array(buffer, 0, scanLen));
  const dataLine = /^[ \t]*DATA[ \t]+(\S+)[ \t]*\r?\n/im.exec(prefix);
  if (!dataLine) {
    throw new Error(
      `no PCD "DATA" line in the first ${scanLen} bytes; this does not look like a PCD file`,
    );
  }
  const headerLen = dataLine.index + dataLine[0].length;
  // Comments may sit anywhere in the header (every PCL-written file opens with
  // a "# .PCD v0.7" banner), so drop them before reading the keyword lines.
  const text = prefix.slice(0, headerLen).replace(/#.*/g, "");

  const fields = words(keywordValue(text, "FIELDS"));
  const size = words(keywordValue(text, "SIZE")).map(Number);
  const type = words(keywordValue(text, "TYPE"));
  const countValue = keywordValue(text, "COUNT");
  const count = countValue ? words(countValue).map(Number) : fields.map(() => 1);
  if (fields.length === 0 || size.length !== fields.length || type.length !== fields.length) {
    throw new Error("PCD header's FIELDS, SIZE and TYPE lines are missing or disagree in length");
  }
  if (count.length !== fields.length) {
    throw new Error("PCD header's COUNT line disagrees with FIELDS in length");
  }

  const pointsValue = keywordValue(text, "POINTS");
  const points = pointsValue
    ? positiveInt(pointsValue, "POINTS")
    : positiveInt(keywordValue(text, "WIDTH"), "WIDTH") *
      positiveInt(keywordValue(text, "HEIGHT"), "HEIGHT");

  const offset: Record<string, number> = {};
  let rowSize = 0;
  for (let i = 0; i < fields.length; i += 1) {
    if (!Number.isSafeInteger(size[i]) || size[i] <= 0 || !Number.isSafeInteger(count[i])) {
      throw new Error(`PCD field "${fields[i]}" has an invalid SIZE/COUNT: ${size[i]}/${count[i]}`);
    }
    offset[fields[i]] = rowSize;
    rowSize += size[i] * count[i];
  }

  return {
    data: dataLine[1].toLowerCase(),
    headerLen,
    fields,
    size,
    type,
    points,
    offset,
    rowSize,
  };
}

// Locate a field and the reader for its declared TYPE/SIZE. Returns null when
// the field is absent; throws when it is present but of a type this cannot read
// (silently dropping x/y/z would render a wrong cloud).
function findField(header: PcdHeader, name: string): PcdField | null {
  const index = header.fields.indexOf(name);
  if (index < 0) {
    return null;
  }
  const read = fieldReader(header.type[index], header.size[index]);
  if (!read) {
    throw new Error(
      `PCD field "${name}" has an unsupported TYPE ${header.type[index]} / SIZE ${header.size[index]}`,
    );
  }
  return { read, offset: header.offset[name] };
}

function requireField(header: PcdHeader, name: string): PcdField {
  const field = findField(header, name);
  if (!field) {
    throw new Error(`PCD has no "${name}" field (FIELDS: ${header.fields.join(" ")})`);
  }
  return field;
}

// An optional scalar: a type this cannot read costs one colouring option, not
// the whole cloud, so report it and carry on without the field.
function optionalField(header: PcdHeader, name: string): PcdField | null {
  try {
    return findField(header, name);
  } catch (error) {
    console.warn(`map viewer: ignoring the PCD's "${name}" field — ${String(error)}`);
    return null;
  }
}

function parseBinary(buffer: ArrayBuffer, header: PcdHeader): PcdCloud {
  const { points, rowSize } = header;
  // Resolve the fields before measuring the payload: a cloud with no z is a
  // different (and more useful) complaint than one that is merely short.
  const x = requireField(header, "x");
  const y = requireField(header, "y");
  const z = requireField(header, "z");
  const intensityField = optionalField(header, "intensity");

  const declared = points * rowSize;
  const available = buffer.byteLength - header.headerLen;
  if (available < declared) {
    throw new Error(
      `truncated PCD: the header declares ${points} points (${declared} bytes) ` +
        `but only ${available} bytes follow the header`,
    );
  }
  // PCL packs r/g/b into the three low bytes of a 4-byte field whatever TYPE it
  // declares (float or uint32), so the bytes are read directly, like PCDLoader.
  const rgbOffset = header.fields.includes("rgb") ? header.offset.rgb : -1;

  const position = new Float32Array(points * 3);
  const intensity = intensityField ? new Float32Array(points) : null;
  const color = rgbOffset >= 0 ? new Float32Array(points * 3) : null;
  const view = new DataView(buffer, header.headerLen);

  for (let i = 0, row = 0; i < points; i += 1, row += rowSize) {
    const at = i * 3;
    position[at] = x.read(view, row + x.offset);
    position[at + 1] = y.read(view, row + y.offset);
    position[at + 2] = z.read(view, row + z.offset);
    if (intensity && intensityField) {
      intensity[i] = intensityField.read(view, row + intensityField.offset);
    }
    if (color) {
      const rgb = row + rgbOffset;
      color[at] = SRGB_BYTE_TO_LINEAR[view.getUint8(rgb + 2)];
      color[at + 1] = SRGB_BYTE_TO_LINEAR[view.getUint8(rgb + 1)];
      color[at + 2] = SRGB_BYTE_TO_LINEAR[view.getUint8(rgb)];
    }
  }

  return { count: points, position, intensity, color };
}

// `ascii` and `binary_compressed` clouds (never written by `map slam`, but the
// viewer opens any -i path) keep going through PCDLoader, which reads both.
function parseWithPcdLoader(buffer: ArrayBuffer, format: string): PcdCloud {
  if (buffer.byteLength > MAX_DECODABLE_BYTES) {
    throw new Error(
      `this DATA ${format} PCD is ${Math.round(buffer.byteLength / 1e6)} MB, past the ` +
        `${Math.round(MAX_DECODABLE_BYTES / 1e6)} MB a browser can decode as text; ` +
        `re-save it as DATA binary, which the viewer reads at any size`,
    );
  }
  const geometry = new PCDLoader().parse(buffer).geometry;
  const position = geometry.getAttribute("position");
  if (!position) {
    throw new Error(`DATA ${format} PCD has no x/y/z fields`);
  }
  const intensity = geometry.getAttribute("intensity");
  const color = geometry.getAttribute("color");
  return {
    count: position.count,
    position: new Float32Array(position.array),
    intensity: intensity ? new Float32Array(intensity.array) : null,
    color: color ? new Float32Array(color.array) : null,
  };
}

// Parse a whole .pcd file. Throws an Error describing the offending header or
// payload; callers surface the message rather than a bare stack trace.
export function parsePcd(buffer: ArrayBuffer): PcdCloud {
  const header = parseHeader(buffer);
  switch (header.data) {
    case "binary":
      return parseBinary(buffer, header);
    case "ascii":
    case "binary_compressed":
      return parseWithPcdLoader(buffer, header.data);
    default:
      throw new Error(`unsupported PCD "DATA ${header.data}" format`);
  }
}
