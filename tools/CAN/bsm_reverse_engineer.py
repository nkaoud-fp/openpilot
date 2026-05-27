#!/usr/bin/env python3
"""
BSM (Blind Spot Monitor) Reverse Engineering Tool for Toyota/Lexus

Analyzes CAN message 0x3F6 (1014) to decode the 58 unknown bits beyond
the 6 currently mapped signals (L_ADJACENT, R_ADJACENT, L_APPROACHING,
R_APPROACHING, ADJACENT_ENABLED, APPROACHING_ENABLED).

Usage:
  Live on comma device:
    python3 bsm_reverse_engineer.py

  From an rlog file:
    python3 bsm_reverse_engineer.py -f /path/to/rlog.bz2

  From a comma connect route:
    python3 bsm_reverse_engineer.py -r "ROUTE_ID"
"""

import argparse
import csv
import os
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime

BSM_CAN_ID = 0x3F6  # 1014 decimal
BSM_MSG_LEN = 8      # 8 bytes = 64 bits

# Known signals (little-endian bit numbering as in the DBC)
KNOWN_SIGNALS = {
    "L_ADJACENT":         (0, 0, 0),   # byte 0, bit 0
    "R_ADJACENT":         (0, 1, 1),   # byte 0, bit 1
    "ADJACENT_ENABLED":   (0, 7, 7),   # byte 0, bit 7
    "L_APPROACHING":      (1, 0, 8),   # byte 1, bit 0
    "R_APPROACHING":      (1, 2, 10),  # byte 1, bit 2
    "APPROACHING_ENABLED":(1, 7, 15),  # byte 1, bit 7
}

# Build bitmask of known bits (byte-level)
KNOWN_BYTE_MASKS = [0x00] * 8
for name, (byte_idx, bit_idx, _) in KNOWN_SIGNALS.items():
    KNOWN_BYTE_MASKS[byte_idx] |= (1 << bit_idx)

# Build 64-bit mask of known bits (big-endian packed)
KNOWN_MASK_64 = 0
for byte_idx, mask in enumerate(KNOWN_BYTE_MASKS):
    KNOWN_MASK_64 |= (mask << (8 * (7 - byte_idx)))

UNKNOWN_MASK_64 = (~KNOWN_MASK_64) & 0xFFFFFFFFFFFFFFFF


def bytes_to_int(data):
    """Convert 8 bytes to a 64-bit integer (big-endian)."""
    return struct.unpack('>Q', data[:8])[0]


def format_binary_annotated(value_64):
    """Format 64 bits with known signals marked."""
    bits = f"{value_64:064b}"
    lines = []
    for byte_idx in range(8):
        byte_bits = bits[byte_idx * 8:(byte_idx + 1) * 8]
        annotations = []
        for bit_pos, ch in enumerate(byte_bits):
            actual_bit = 7 - bit_pos  # MSB first in display
            for name, (bi, bbit, _) in KNOWN_SIGNALS.items():
                if bi == byte_idx and bbit == actual_bit:
                    annotations.append(f"  bit {actual_bit}: {ch} <- {name}")
        line = f"  byte {byte_idx}: {byte_bits}  (0x{int(byte_bits, 2):02X})"
        lines.append(line)
        for a in annotations:
            lines.append(a)
    return "\n".join(lines)


class BSMAnalyzer:
    def __init__(self):
        self.samples = []
        self.timestamps = []
        self.known_states = []
        self.transition_log = []

        # Tracking per-bit statistics
        self.bit_high_count = [0] * 64
        self.bit_low_count = [0] * 64
        self.total_count = 0

        # For change detection
        self.prev_value = None
        self.prev_unknown = None

        # Per-bit: list of (timestamp, old_val, new_val)
        self.bit_transitions = defaultdict(list)

        # Track correlation with known signals
        self.state_to_unknown_values = defaultdict(list)

    def add_sample(self, data_bytes, timestamp=None):
        if len(data_bytes) < 8:
            return

        ts = timestamp or time.monotonic()
        value = bytes_to_int(data_bytes)
        unknown_value = value & UNKNOWN_MASK_64

        self.samples.append(value)
        self.timestamps.append(ts)
        self.total_count += 1

        # Extract known signal state
        known = {}
        for name, (byte_idx, bit_idx, _) in KNOWN_SIGNALS.items():
            known[name] = (data_bytes[byte_idx] >> bit_idx) & 1
        self.known_states.append(known)

        # Build a state key from known signals for correlation
        state_key = tuple(sorted(known.items()))
        self.state_to_unknown_values[state_key].append(unknown_value)

        # Per-bit high/low counting
        for bit in range(64):
            if (value >> (63 - bit)) & 1:
                self.bit_high_count[bit] += 1
            else:
                self.bit_low_count[bit] += 1

        # Detect transitions in unknown bits
        if self.prev_value is not None:
            changed = (value ^ self.prev_value) & UNKNOWN_MASK_64
            if changed:
                for bit in range(64):
                    if (changed >> (63 - bit)) & 1:
                        old = (self.prev_value >> (63 - bit)) & 1
                        new = (value >> (63 - bit)) & 1
                        self.bit_transitions[bit].append((ts, old, new))

                self.transition_log.append({
                    'timestamp': ts,
                    'known_state': known,
                    'prev_raw': f"{self.prev_value:016X}",
                    'curr_raw': f"{value:016X}",
                    'changed_bits': changed,
                })

        self.prev_value = value
        self.prev_unknown = unknown_value

    def report(self):
        if self.total_count == 0:
            print("No BSM samples collected.")
            return

        print("\n" + "=" * 70)
        print("  BSM (0x3F6) REVERSE ENGINEERING REPORT")
        print("=" * 70)
        print(f"  Total samples: {self.total_count}")
        print(f"  Unique raw values: {len(set(self.samples))}")
        duration = self.timestamps[-1] - self.timestamps[0] if len(self.timestamps) > 1 else 0
        print(f"  Duration: {duration:.1f}s")
        print(f"  Avg rate: {self.total_count / max(duration, 0.001):.1f} msg/s")

        # --- Bit-level analysis ---
        print("\n" + "-" * 70)
        print("  BIT-LEVEL ANALYSIS (unknown bits only)")
        print("-" * 70)
        print(f"  {'Byte':>4} {'Bit':>4} {'DBC#':>5} {'High%':>7} {'Transitions':>12}  Classification")
        print(f"  {'----':>4} {'---':>4} {'----':>5} {'-----':>7} {'-----------':>12}  --------------")

        interesting_bits = []
        for byte_idx in range(8):
            for bit_in_byte in range(7, -1, -1):  # MSB to LSB
                global_bit = byte_idx * 8 + (7 - bit_in_byte)
                dbc_bit = byte_idx * 8 + bit_in_byte

                # Skip known signals
                is_known = False
                known_name = ""
                for name, (bi, bbit, _) in KNOWN_SIGNALS.items():
                    if bi == byte_idx and bbit == bit_in_byte:
                        is_known = True
                        known_name = name
                        break

                high = self.bit_high_count[global_bit]
                total = high + self.bit_low_count[global_bit]
                pct = (high / total * 100) if total > 0 else 0
                transitions = len(self.bit_transitions.get(global_bit, []))

                if is_known:
                    classification = f"KNOWN ({known_name})"
                elif pct == 0:
                    classification = "always 0"
                elif pct == 100:
                    classification = "always 1"
                elif transitions == 0:
                    classification = "static"
                elif transitions <= 3:
                    classification = "** TOGGLE (rare) **"
                    interesting_bits.append((byte_idx, bit_in_byte, dbc_bit, pct, transitions))
                elif pct > 40 and pct < 60:
                    classification = "** COUNTER/DATA? **"
                    interesting_bits.append((byte_idx, bit_in_byte, dbc_bit, pct, transitions))
                else:
                    classification = f"** ACTIVE ({transitions} changes) **"
                    interesting_bits.append((byte_idx, bit_in_byte, dbc_bit, pct, transitions))

                marker = " " if is_known else "*" if transitions > 0 else " "
                print(f" {marker}byte{byte_idx:>1} bit{bit_in_byte:>1} dbc{dbc_bit:>2} {pct:>6.1f}% {transitions:>12}  {classification}")

        # --- Byte-level analysis ---
        print("\n" + "-" * 70)
        print("  BYTE-LEVEL ANALYSIS")
        print("-" * 70)

        byte_values = defaultdict(set)
        for sample in self.samples:
            for byte_idx in range(8):
                byte_val = (sample >> (8 * (7 - byte_idx))) & 0xFF
                unknown_byte = byte_val & ~KNOWN_BYTE_MASKS[byte_idx]
                byte_values[byte_idx].add(unknown_byte)

        for byte_idx in range(8):
            vals = sorted(byte_values[byte_idx])
            known_mask = KNOWN_BYTE_MASKS[byte_idx]
            unknown_mask = (~known_mask) & 0xFF
            print(f"\n  Byte {byte_idx} (known mask: 0x{known_mask:02X}, unknown mask: 0x{unknown_mask:02X}):")
            print(f"    Unique unknown values: {len(vals)}")
            if len(vals) <= 32:
                vals_hex = [f"0x{v:02X}" for v in vals]
                for i in range(0, len(vals_hex), 8):
                    print(f"      {', '.join(vals_hex[i:i+8])}")
            else:
                print(f"      Range: 0x{min(vals):02X} - 0x{max(vals):02X} ({len(vals)} unique values)")
                print(f"      (likely multi-bit counter or distance value)")

        # --- Correlation analysis ---
        print("\n" + "-" * 70)
        print("  CORRELATION: unknown bits vs. known BSM state")
        print("-" * 70)

        for state_key, unknown_vals in sorted(self.state_to_unknown_values.items()):
            state_dict = dict(state_key)
            active_signals = [n for n, v in state_dict.items() if v == 1]
            state_label = ", ".join(active_signals) if active_signals else "(all clear)"
            unique_unknown = set(unknown_vals)
            print(f"\n  Known state: {state_label}")
            print(f"    Occurrences: {len(unknown_vals)}")
            print(f"    Unique unknown patterns: {len(unique_unknown)}")
            if len(unique_unknown) <= 8:
                for uv in sorted(unique_unknown):
                    print(f"      0x{uv:016X}")

        # --- Transition log ---
        if self.transition_log:
            print("\n" + "-" * 70)
            print(f"  TRANSITION LOG (unknown bits changed) -- {len(self.transition_log)} events")
            print("-" * 70)
            for i, entry in enumerate(self.transition_log[:50]):
                active = [n for n, v in entry['known_state'].items() if v == 1]
                state_label = ", ".join(active) if active else "(clear)"
                changed_bits_str = f"0x{entry['changed_bits']:016X}"
                t = entry['timestamp']
                print(f"  [{t:>10.2f}s] {entry['prev_raw']} -> {entry['curr_raw']}  changed: {changed_bits_str}  state: {state_label}")
            if len(self.transition_log) > 50:
                print(f"  ... and {len(self.transition_log) - 50} more transitions")

        # --- Interesting bits summary ---
        if interesting_bits:
            print("\n" + "-" * 70)
            print("  INTERESTING UNKNOWN BITS (candidates for new signals)")
            print("-" * 70)
            for byte_idx, bit_in_byte, dbc_bit, pct, transitions in interesting_bits:
                print(f"    byte {byte_idx}, bit {bit_in_byte} (DBC bit {dbc_bit}): "
                      f"{pct:.1f}% high, {transitions} transitions")

        # --- Multi-bit field candidates ---
        print("\n" + "-" * 70)
        print("  MULTI-BIT FIELD CANDIDATES")
        print("-" * 70)
        print("  Looking for contiguous unknown bits that change together...\n")

        for byte_idx in range(8):
            unknown_mask = (~KNOWN_BYTE_MASKS[byte_idx]) & 0xFF
            if unknown_mask == 0:
                continue

            byte_samples = []
            for sample in self.samples:
                byte_val = (sample >> (8 * (7 - byte_idx))) & 0xFF
                byte_samples.append(byte_val & unknown_mask)

            unique_vals = sorted(set(byte_samples))
            if len(unique_vals) > 2:
                # Check for contiguous bit groups
                contiguous = []
                start = None
                for bit in range(8):
                    if unknown_mask & (1 << bit):
                        if start is None:
                            start = bit
                        end = bit
                    else:
                        if start is not None:
                            contiguous.append((start, end))
                            start = None
                if start is not None:
                    contiguous.append((start, end))

                for start_bit, end_bit in contiguous:
                    width = end_bit - start_bit + 1
                    if width > 1:
                        field_mask = ((1 << width) - 1) << start_bit
                        field_vals = sorted(set((v & field_mask) >> start_bit for v in byte_samples))
                        if len(field_vals) > 2:
                            dbc_start = byte_idx * 8 + start_bit
                            print(f"  Byte {byte_idx}, bits [{end_bit}:{start_bit}] "
                                  f"(DBC bits [{byte_idx*8+end_bit}:{dbc_start}], width={width}):")
                            print(f"    {len(field_vals)} unique values: "
                                  f"{', '.join(str(v) for v in field_vals[:20])}"
                                  f"{'...' if len(field_vals) > 20 else ''}")
                            if max(field_vals) - min(field_vals) == len(field_vals) - 1:
                                print(f"    *** Looks like a COUNTER (sequential) ***")
                            else:
                                val_diffs = [field_vals[i+1] - field_vals[i] for i in range(len(field_vals)-1)]
                                if len(set(val_diffs)) <= 3:
                                    print(f"    *** Regular spacing -- possible DISTANCE or LEVEL ***")

        print("\n" + "=" * 70)
        print("  END OF REPORT")
        print("=" * 70)

    def export_csv(self, filename="bsm_raw_data.csv"):
        """Export raw samples to CSV for external analysis."""
        if not self.samples:
            print("No data to export.")
            return

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp', 'raw_hex']
            for byte_idx in range(8):
                header.append(f'byte{byte_idx}')
            for name in sorted(KNOWN_SIGNALS.keys()):
                header.append(name)
            writer.writerow(header)

            for i, (sample, ts) in enumerate(zip(self.samples, self.timestamps)):
                row = [f"{ts:.3f}", f"{sample:016X}"]
                for byte_idx in range(8):
                    byte_val = (sample >> (8 * (7 - byte_idx))) & 0xFF
                    row.append(f"0x{byte_val:02X}")
                for name in sorted(KNOWN_SIGNALS.keys()):
                    row.append(self.known_states[i].get(name, ''))
                writer.writerow(row)

        print(f"Exported {len(self.samples)} samples to {filename}")


def monitor_live(analyzer, duration=None):
    """Monitor BSM on live CAN bus (run on comma device)."""
    import cereal.messaging as messaging

    can_sock = messaging.sub_sock('can')
    print(f"\nMonitoring BSM (0x3F6) on live CAN bus...")
    if duration:
        print(f"Will run for {duration} seconds.")
    print("Press Ctrl+C to stop and generate report.\n")

    start = time.monotonic()
    last_print = 0
    prev_known_state = None
    first_raw = None

    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
            for msg in can_msgs:
                for can_msg in msg.can:
                    if can_msg.address == BSM_CAN_ID and len(can_msg.dat) == BSM_MSG_LEN:
                        data = bytes(can_msg.dat)
                        elapsed = time.monotonic() - start
                        analyzer.add_sample(data, elapsed)

                        if first_raw is None:
                            first_raw = data
                            print(f"  First raw message: {data.hex()}")
                            print(f"  Byte breakdown:")
                            for bi in range(8):
                                known_mask = KNOWN_BYTE_MASKS[bi]
                                unknown_mask = (~known_mask) & 0xFF
                                known_val = data[bi] & known_mask
                                unknown_val = data[bi] & unknown_mask
                                print(f"    byte {bi}: 0x{data[bi]:02X} "
                                      f"(known=0x{known_val:02X} unknown=0x{unknown_val:02X} "
                                      f"bits={data[bi]:08b})")
                            print()

                        known = {}
                        for name, (byte_idx, bit_idx, _) in KNOWN_SIGNALS.items():
                            known[name] = (data[byte_idx] >> bit_idx) & 1
                        known_key = tuple(sorted(known.items()))

                        if prev_known_state is not None and known_key != prev_known_state:
                            active = [n for n, v in known.items() if v == 1]
                            label = ", ".join(active) if active else "(all clear)"
                            print(f"\n  [{elapsed:.1f}s] BSM STATE CHANGED -> {label}")
                            print(f"    raw: {data.hex()}")
                            for bi in range(8):
                                known_mask = KNOWN_BYTE_MASKS[bi]
                                unknown_mask = (~known_mask) & 0xFF
                                unknown_val = data[bi] & unknown_mask
                                if unknown_val != (first_raw[bi] & unknown_mask):
                                    print(f"    ** byte {bi} unknown changed: "
                                          f"0x{first_raw[bi] & unknown_mask:02X} -> 0x{unknown_val:02X}")
                            print()

                        prev_known_state = known_key

            now = time.monotonic()
            if now - last_print > 2.0:
                last_print = now
                elapsed = now - start
                print(f"\r  [{elapsed:.0f}s] Samples: {analyzer.total_count}, "
                      f"Transitions: {len(analyzer.transition_log)}", end="", flush=True)

            if duration and (time.monotonic() - start) > duration:
                break

    except KeyboardInterrupt:
        pass

    print("\n")


def parse_rlog(analyzer, filepath):
    """Parse BSM messages from an rlog file."""
    from openpilot.tools.lib.logreader import LogReader

    print(f"\nParsing: {filepath}")
    lr = LogReader(filepath)

    for msg in lr:
        if msg.which() == 'can':
            for can_msg in msg.can:
                if can_msg.address == BSM_CAN_ID and len(can_msg.dat) == BSM_MSG_LEN:
                    analyzer.add_sample(bytes(can_msg.dat), msg.logMonoTime / 1e9)

    print(f"  Found {analyzer.total_count} BSM messages")


def parse_route(analyzer, route_id):
    """Parse BSM messages from a comma connect route."""
    from openpilot.tools.lib.logreader import LogReader
    from openpilot.tools.lib.route import Route

    print(f"\nFetching route: {route_id}")
    route = Route(route_id)
    log_paths = route.log_paths()

    for seg_idx, log_path in enumerate(log_paths):
        if log_path is None:
            continue
        print(f"  Segment {seg_idx}...", end=" ", flush=True)
        try:
            lr = LogReader(log_path)
            seg_count = 0
            for msg in lr:
                if msg.which() == 'can':
                    for can_msg in msg.can:
                        if can_msg.address == BSM_CAN_ID and len(can_msg.dat) == BSM_MSG_LEN:
                            analyzer.add_sample(bytes(can_msg.dat), msg.logMonoTime / 1e9)
                            seg_count += 1
            print(f"{seg_count} BSM messages")
        except Exception as e:
            print(f"error: {e}")

    print(f"\nTotal: {analyzer.total_count} BSM messages across route")


def main():
    parser = argparse.ArgumentParser(
        description="Reverse-engineer undecoded bits in Toyota BSM CAN message 0x3F6",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Live monitoring on comma device:
    python3 bsm_reverse_engineer.py
    python3 bsm_reverse_engineer.py --duration 300

  From a local rlog file:
    python3 bsm_reverse_engineer.py -f /data/media/0/realdata/ROUTE--SEG/rlog.bz2

  From a comma connect route:
    python3 bsm_reverse_engineer.py -r "a]b]c|2024-01-15--12-00-00"

  Export raw data to CSV:
    python3 bsm_reverse_engineer.py -f /path/to/rlog.bz2 --csv bsm_data.csv
        """,
    )
    parser.add_argument("-f", "--file", help="Path to rlog file")
    parser.add_argument("-r", "--route", help="Comma connect route ID")
    parser.add_argument("--duration", type=int, help="Live monitoring duration in seconds")
    parser.add_argument("--csv", help="Export raw data to CSV file", default=None)
    args = parser.parse_args()

    analyzer = BSMAnalyzer()

    if args.file:
        parse_rlog(analyzer, args.file)
    elif args.route:
        parse_route(analyzer, args.route)
    else:
        monitor_live(analyzer, args.duration)

    analyzer.report()

    if args.csv:
        analyzer.export_csv(args.csv)
    elif analyzer.total_count > 0:
        default_csv = "bsm_raw_data.csv"
        analyzer.export_csv(default_csv)


if __name__ == "__main__":
    main()
