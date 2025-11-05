#!/usr/bin/env python3
import os
import time
import cereal.messaging as messaging
from collections import defaultdict

# --- Configuration ---
# How often to refresh the dashboard on the screen
REFRESH_RATE_SECONDS = 0.5

def main():
  os.system('clear')
  print("="*60)
  print("             ✅ Live CAN Data Differ ✅")
  print("="*60)
  print("\nThis script finds new data payloads, even on busy CAN IDs.")
  print("The screen will update periodically with new findings.")
  print("\n>>> Perform actions in your car to discover new commands. <<<")
  print("-" * 60)
  time.sleep(2)

  can_sock = messaging.sub_sock('can')
  
  # A dictionary where keys are CAN IDs and values are sets of unique data payloads
  seen_messages = defaultdict(set)
  last_update_time = 0

  try:
    while True:
      # --- Data Collection Phase ---
      can_msgs = messaging.drain_sock(can_sock)
      for msg in can_msgs:
        for can_msg in msg.can:
          # Add the new data to the set for its ID.
          # Sets automatically handle duplicates.
          seen_messages[can_msg.address].add(can_msg.dat)
      
      # --- Display Phase (updates periodically) ---
      current_time = time.monotonic()
      if current_time - last_update_time > REFRESH_RATE_SECONDS:
        last_update_time = current_time
        os.system('clear')
        print("--- Live CAN Data Differ --- | Press Ctrl+C to stop")
        print("--- Displaying unique data payloads for each ID ---")
        print("-" * 60)
        
        # Sort and print the dictionary for a stable display
        for can_id in sorted(seen_messages.keys()):
          # Only display IDs that have messages
          if seen_messages[can_id]:
            print(f"ID: {hex(can_id)} ({can_id}) - Found {len(seen_messages[can_id])} unique message(s):")
            for data in sorted(list(seen_messages[can_id])):
              print(f"  - Data: {data.hex()}")
        
  except KeyboardInterrupt:
    print("\n\n" + "="*60)
    print("Sniffer stopped by user.")
    print(f"Discovered {len(seen_messages)} unique CAN IDs in total.")
    print("="*60)

if __name__ == "__main__":
    main()