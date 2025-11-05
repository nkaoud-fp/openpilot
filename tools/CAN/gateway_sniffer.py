#!/usr/bin/env python3
import os
import re
import time
from collections import defaultdict

import cereal.messaging as messaging

#
## HOW TO RUN:
##   python3 /data/dbc_maker.py
#
#  This script provides a real-time dashboard of raw CAN messages.
#  It intelligently highlights messages when their data patterns change
#  and allows you to select specific message IDs to monitor.
#

# Using the 1.2 second delay from your saved preferences for the live analysis refresh rate.
LIVE_REFRESH_RATE = 1.2

def print_header(title):
    os.system('clear')
    print("="*70)
    print(f"| {title.center(66)} |")
    print("="*70)

def to_motorola_bit(bit_index):
    byte_num, bit_in_byte = bit_index // 8, bit_index % 8
    return byte_num * 8 + (7 - bit_in_byte)

def get_signals_from_mask(mask, max_dlc):
    """
    Calculates a list of guessed signals from a bitmask.
    This is used internally to detect when a message's data pattern changes.
    """
    signals = []
    max_bits = max_dlc * 8
    in_signal = False
    start_lsb = 0
    for i in range(max_bits + 1):
        is_set = (mask >> i) & 1
        if is_set and not in_signal:
            in_signal = True
            start_lsb = i
        elif not is_set and in_signal:
            in_signal = False
            end_msb = i - 1
            length = end_msb - start_lsb + 1
            start_bit_motorola = to_motorola_bit(end_msb)
            signals.append({'start_bit': start_bit_motorola, 'length': length, 'name': f'SIG_{start_bit_motorola}'})
    return signals

def main():
    print_header("Live CAN Raw Data Monitor")

    # NEW: Interactive prompt for selecting message IDs to monitor
    monitored_ids = set()
    input_str = input("Enter CAN IDs to monitor (e.g., 512, 740, 0x365). Leave blank for all: ").strip()
    if input_str:
        try:
            # Handle both decimal and hex (e.g., '0x200') by using base 0
            id_list = [int(i.strip(), 0) for i in input_str.split(',')]
            monitored_ids = set(id_list)
            print(f"✅ Monitoring {len(monitored_ids)} specific ID(s): {sorted(list(monitored_ids))}")
        except ValueError:
            print("❌ Invalid input. Please enter numbers (e.g., 512, 0x200). Monitoring all messages as a fallback.")
            monitored_ids = set() # Fallback to monitoring all
    else:
        print("✅ Monitoring all messages.")

    print("\nStarting live analysis... Press Ctrl+C to stop.")
    time.sleep(2)

    can_sock = messaging.sub_sock('can')
    live_state = defaultdict(lambda: {'payloads': [], 'last_mask': 0, 'latest_payload': b'', 'last_change_time': 0})
    last_update_time = 0

    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock)
            for msg in can_msgs:
                for can_msg in msg.can:
                    msg_id = can_msg.address
                    # NEW: Filter for specific IDs. If monitored_ids is empty, this passes for all messages.
                    if not monitored_ids or msg_id in monitored_ids:
                        live_state[msg_id]['payloads'].append(can_msg.dat)
                        live_state[msg_id]['latest_payload'] = can_msg.dat

            current_time = time.monotonic()
            if current_time - last_update_time < LIVE_REFRESH_RATE:
                continue
            last_update_time = current_time

            structure_changed = False
            for msg_id, data in live_state.items():
                if len(data['payloads']) < 2:
                    continue

                max_dlc = max(len(p) for p in data['payloads'])
                first_msg_int = int.from_bytes(data['payloads'][0].ljust(max_dlc, b'\x00'), 'big')
                current_mask = 0
                for p in data['payloads'][1:]:
                    current_mask |= (first_msg_int ^ int.from_bytes(p.ljust(max_dlc, b'\x00'), 'big'))

                if current_mask != data['last_mask']:
                    structure_changed = True
                    data['last_mask'] = current_mask
                    data['last_change_time'] = current_time

            if structure_changed:
                print_header("Live CAN Raw Data Monitor")
                for msg_id in sorted(live_state.keys()):
                    data = live_state[msg_id]
                    if not data['latest_payload']:
                        continue

                    change_indicator = ""
                    if current_time - data['last_change_time'] < (LIVE_REFRESH_RATE + 0.5):
                       change_indicator = "    <-- UPDATED"

                    print(f"\n--- ID: {msg_id} (0x{msg_id:X}) ---{change_indicator}")
                    payload_str = ''.join(f'\\x{b:02x}' for b in data['latest_payload'])
                    print(f"  Data: {payload_str}")

    except KeyboardInterrupt:
        print("\n\n--- Monitoring Stopped by User ---")
        print("Exiting.")

if __name__ == "__main__":
    main()