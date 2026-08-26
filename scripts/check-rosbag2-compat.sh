#!/usr/bin/env bash
# Verify that bags bagwiz writes can be opened by the ROS 2 distro it was built
# against, in both layouts.
#
# This guards a failure mode unit tests cannot see. rosbag2 selects its metadata
# YAML decoder from the declared `version`, and jazzy+ parses that metadata while
# probing whether a storage plugin can open the file. A document whose structure
# disagrees with its version therefore does not merely report wrong numbers — the
# bag stops being openable at all:
#
#   [ERROR] [rosbag2_storage]: No storage id specified, and no plugin found
#   that could open URI: '...'
#
# bagwiz pins `version: 5` (the humble baseline) on every distro so one output
# shape stays readable everywhere: rosbag2 reads older metadata forward but not
# newer metadata backward, and bagwiz hands its output to consumers whose distro
# it does not control.
#
# The seed bag is built with plain sqlite3 rather than `ros2 bag record` so the
# check needs no ROS graph, no DDS and no publisher.
set -euo pipefail

BAGWIZ_BIN="${BAGWIZ_BIN:-./build/${PIXI_ENVIRONMENT_NAME:-default}/bagwiz/bagwiz}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

if [[ ! -x $BAGWIZ_BIN ]]; then
    echo "check-rosbag2-compat: bagwiz binary not found at $BAGWIZ_BIN" >&2
    exit 1
fi

SEED="$WORK/seed.db3"
# CDR-encapsulated std_msgs/String carrying "hi". Held in hex so the read-back
# check can compare against the exact bytes the seed went in with.
SEED_PAYLOAD_HEX="0001000003000000686900"
python3 - "$SEED" "$SEED_PAYLOAD_HEX" <<'PY'
import sqlite3, sys

db = sqlite3.connect(sys.argv[1])
db.executescript("""
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE metadata(id INTEGER PRIMARY KEY, metadata_version INTEGER NOT NULL,
                      metadata TEXT NOT NULL);
CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                    serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
                      timestamp INTEGER NOT NULL, data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages (timestamp ASC);
INSERT INTO schema VALUES (3, 'humble');
INSERT INTO topics VALUES (1, '/chatter', 'std_msgs/msg/String', 'cdr', '');
""")
payload = bytes.fromhex(sys.argv[2])
db.executemany(
    "INSERT INTO messages(topic_id, timestamp, data) VALUES (1, ?, ?)",
    [(1_700_000_000_000_000_000 + i * 1_000_000, payload) for i in range(5)])
db.commit()
db.close()
PY

fail=0

# Indent a captured block onto stderr so failure detail reads as sub-output.
indent() { echo "      ${1//$'\n'/$'\n'      }" >&2; }

check() { # <label> <path>
    local label="$1" path="$2" out detail
    if ! out="$(ros2 bag info "$path" 2>&1)"; then
        echo "FAIL  $label: ros2 bag info exited non-zero" >&2
        indent "$out"
        fail=1
        return
    fi
    if grep -qE "No plugin detected|Exception on parsing|Error opening" <<<"$out"; then
        echo "FAIL  $label: ros2 bag info could not read the bag" >&2
        detail="$(grep -iE "error|exception|no plugin" <<<"$out" || true)"
        indent "$detail"
        fail=1
        return
    fi
    if ! grep -qE "Messages: +5( |$)" <<<"$out"; then
        echo "FAIL  $label: expected 5 messages" >&2
        indent "$out"
        fail=1
        return
    fi
    echo "ok    $label"
}

# `ros2 bag info` summarises metadata.yaml and probes for a storage plugin, but
# it never asks rosbag2 to decode a message. That blind spot matters for the
# compressed shapes `bagwiz compress` writes: rosbag2 picks its reader from the
# metadata's `compression_mode`, so a bag can summarise perfectly and still be
# unopenable by `ros2 bag play`. Round-tripping through `ros2 bag convert`
# exercises the reader rosbag2 would really use, and comparing the payload back
# out proves the decompression itself agreed with what bagwiz wrote.
readback() { # <label> <path>
    local label="$1" path="$2" dest cfg out got
    dest="$WORK/readback_${label//[^[:alnum:]]/_}"
    cfg="$dest.yaml"
    rm -rf "$dest"
    cat >"$cfg" <<YML
output_bags:
  - uri: $dest
    storage_id: sqlite3
    all_topics: true
    all_services: true
YML
    if ! out="$(ros2 bag convert -i "$path" -o "$cfg" 2>&1)"; then
        echo "FAIL  $label: ros2 bag convert could not read the bag" >&2
        indent "$out"
        fail=1
        return
    fi
    if ! got="$(
        python3 - "$dest" "$SEED_PAYLOAD_HEX" <<'PY'
import glob, os, sqlite3, sys

shards = sorted(glob.glob(os.path.join(sys.argv[1], "*.db3")))
if not shards:
    print("no .db3 shard produced")
    raise SystemExit(1)
db = sqlite3.connect(shards[0])
rows = db.execute("SELECT data FROM messages ORDER BY timestamp").fetchall()
if len(rows) != 5:
    print(f"expected 5 messages, decoded {len(rows)}")
    raise SystemExit(1)
wrong = {bytes(r[0]).hex() for r in rows} - {sys.argv[2]}
if wrong:
    print(f"payload mismatch after decompression: {sorted(wrong)}")
    raise SystemExit(1)
PY
    )"; then
        echo "FAIL  $label: $got" >&2
        fail=1
        return
    fi
    echo "ok    $label (rosbag2 read-back)"
}

# `convert format` refuses a no-op (same storage AND same layout), so reach the
# directory bag by changing layout, then the single file by changing it back.
# Both hops are genuine bagwiz writes.
"$BAGWIZ_BIN" convert format -i "$SEED" -o "$WORK/dir" >/dev/null
check "directory bag" "$WORK/dir"

"$BAGWIZ_BIN" convert format -i "$WORK/dir" -o "$WORK/single.db3" >/dev/null
check "single-file .db3" "$WORK/single.db3"

"$BAGWIZ_BIN" convert format -i "$SEED" -o "$WORK/single.mcap" >/dev/null
# humble ships no mcap storage plugin at all. That is a distro limitation
# rather than a bagwiz regression, so probe for it and skip. Capture the output
# first: `ros2 bag info` exits non-zero here, and under `pipefail` piping it
# straight into grep would report the pipeline as failed even on a match.
mcap_probe="$(ros2 bag info "$WORK/single.mcap" 2>&1 || true)"
# Distinguish "this distro has no mcap plugin" from "the mcap plugin exists but
# rejected our file" — only the latter is a bagwiz regression. When rosbag2
# cannot match a plugin it also logs the ones it does have; humble's list has no
# mcap entry, jazzy's does.
have_mcap_plugin=1
if grep -q "No plugin detected" <<<"$mcap_probe" &&
    ! grep -q "Available storage plugins:.*mcap" <<<"$mcap_probe"; then
    have_mcap_plugin=0
fi

if ((have_mcap_plugin)); then
    check "single-file .mcap" "$WORK/single.mcap"
else
    echo "skip  single-file .mcap (no mcap storage plugin on this distro)"
fi

# Every compressed shape `bagwiz compress` can write. Each one changes how
# rosbag2 must open the bag, so each is read all the way back.
"$BAGWIZ_BIN" compress -i "$SEED" -o "$WORK/sqlite3_message" --storage sqlite3 >/dev/null
check "sqlite3 MESSAGE-mode" "$WORK/sqlite3_message"
readback "sqlite3 MESSAGE-mode" "$WORK/sqlite3_message"

"$BAGWIZ_BIN" compress -i "$SEED" -o "$WORK/sqlite3_file" --storage sqlite3 --mode file >/dev/null
check "sqlite3 FILE-mode envelope" "$WORK/sqlite3_file"
readback "sqlite3 FILE-mode envelope" "$WORK/sqlite3_file"

for codec in zstd lz4; do
    "$BAGWIZ_BIN" compress -i "$SEED" -o "$WORK/mcap_$codec" --storage mcap --codec "$codec" \
        >/dev/null
    if ((have_mcap_plugin)); then
        check "mcap $codec chunks" "$WORK/mcap_$codec"
        readback "mcap $codec chunks" "$WORK/mcap_$codec"
    else
        echo "skip  mcap $codec chunks (no mcap storage plugin on this distro)"
    fi
done

exit "$fail"
