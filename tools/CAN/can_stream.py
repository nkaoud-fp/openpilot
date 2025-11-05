#!/usr/bin/env python3
import sys
import cereal.messaging as messaging

#python can_stream.py | python stream_filter.py

def main():
  """
  A high-speed CAN data streamer. It does no processing, only printing.
  This script is intended to be piped to other command-line tools like 'grep'.
  """
  can_sock = messaging.sub_sock('can')
  while True:
    can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
    for msg in can_msgs:
      for can_msg in msg.can:
        # Print a simple, consistent format for easy parsing
        print(f"BUS: {can_msg.src}  ID: {hex(can_msg.address)} ({can_msg.address})  Data: {can_msg.dat.hex()}")
    # Flush the output buffer to ensure the pipe receives data immediately
    sys.stdout.flush()

if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass