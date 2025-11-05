#!/usr/bin/env python3
import os
import time
import cereal.messaging as messaging
from datetime import datetime

# --- Configuration ---
# An ID is considered "returning" if it hasn't been seen for this many seconds
ROLLING_WINDOW_SECONDS = 10.0

def main():
  os.system('clear')
  print("="*60)
  print("             ✅ Rolling CAN Sniffer ✅")
  print("="*60)
  print(f"\nMonitoring for new and returning CAN IDs.")
  print(f"An ID is 'returning' if it's been silent for >{int(ROLLING_WINDOW_SECONDS)} seconds.")
  print("\n>>> Perform actions in your car to see event-based messages. <<<")
  print("-" * 60)

  can_sock = messaging.sub_sock('can')
  
  # A dictionary to store the last time we saw each CAN ID
  seen_ids_timestamps = {}

  try:
    while True:
      can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
      current_time = time.monotonic()
      timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
      
      for msg in can_msgs:
        for can_msg in msg.can:
          address = can_msg.address
          last_seen_time = seen_ids_timestamps.get(address)

          # Case 1: This is a brand new ID we have never seen before
          if last_seen_time is None:
            print(f"[{timestamp}] ✨ NEW ID       -> ID: {hex(address)} ({address})  Data: {can_msg.dat.hex()}")
          
          # Case 2: This is an old ID that has reappeared after being silent
          elif current_time - last_seen_time > ROLLING_WINDOW_SECONDS:
            print(f"[{timestamp}] ↪️ RETURNING ID -> ID: {hex(address)} ({address})  Data: {can_msg.dat.hex()}")

          # Always update the timestamp for the ID to keep it "fresh"
          seen_ids_timestamps[address] = current_time

  except KeyboardInterrupt:
    print("\n\n" + "="*60)
    print("Sniffer stopped by user.")
    print("="*60)

if __name__ == "__main__":
    main()