#!/usr/bin/env python3
import cereal.messaging as messaging
from datetime import datetime

# --- Define the command "patterns" (data payload excluding the first byte) ---
PATTERNS = {
    "MIRROR_FOLD":   b"\x04\x30\x21\x00\x08\x00\x00",
    "MIRROR_UNFOLD": b"\x04\x30\x21\x00\x04\x00\x00",
    "WINDOW_CLOSE":  b"\x04\x30\x01\x05\x20\x00\x00",
    "WINDOW_OPEN":   b"\x04\x30\x01\x05\x10\x00\x00",
}

def main():
  # --- Instructions ---
  print("="*60)
  print("      Flexible Command Pattern Sniffer")
  print("="*60)
  print("\nThis script is now listening for any messages that match these patterns,")
  print("regardless of the Gateway ID or Target Address:")
  for name, pattern in PATTERNS.items():
      print(f"  - {name:<14}: ...{pattern.hex()}")
      
  print("\n>>> Please press the 'Fold All Mirrors' button in your car. <<<")
  print("    (You can also try other buttons like window close/open)")
  print("\nPress Ctrl+C to stop sniffing.")
  print("-" * 60)

  # --- Main Loop ---
  can_sock = messaging.sub_sock('can')
  try:
    while True:
      can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)

      for msg in can_msgs:
        for can_msg in msg.can:
          # We only care about 8-byte messages for this test
          if len(can_msg.dat) == 8:
            # The pattern is the last 7 bytes of the data
            msg_pattern = can_msg.dat[1:]

            # Check if this pattern matches any we know
            for pattern_name, pattern_data in PATTERNS.items():
              if msg_pattern == pattern_data:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                gateway_id = can_msg.address
                target_addr = can_msg.dat[0]

                print(f"[{timestamp}] >>> MATCH FOUND! ({pattern_name}) <<<")
                print(f"  - Gateway/Sender ID : {hex(gateway_id)} ({gateway_id})")
                print(f"  - Target Address    : {hex(target_addr)} ({target_addr})")
                print(f"  - Full Data         : {can_msg.dat.hex()}")
                print("-" * 60)

  except KeyboardInterrupt:
    print("\n" + "="*60)
    print("Sniffing stopped by user.")
    print("="*60)

if __name__ == "__main__":
    main()