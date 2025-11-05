import cereal.messaging as messaging
from opendbc.can.parser import CANParser
import time
import os

# --- Configuration ---
# The name of the DBC file, without the .dbc extension
DBC_NAME = 'toyota_nodsu_pt_generated'

# The message and signals we are interested in
MESSAGE_TO_MONITOR = 'DOOR_LOCKS'
CONDITION_SIGNAL = 'LOCK_STATUS_CHANGED'
TARGET_SIGNAL = 'UNKNOWN_183'

# Refresh rate in seconds
REFRESH_DELAY = 1.0

def monitor_signal_status():
    """
    Monitors a specific CAN signal based on a condition from another signal
    and displays its status in the terminal.
    """
    # 1. Setup the CAN Parser
    # We tell the parser which message we want to receive (DOOR_LOCKS) and how often we expect it (e.g., 10 times a second)
    messages = [(MESSAGE_TO_MONITOR, 10)]
    can_parser = CANParser(DBC_NAME, messages, 0)

    # 2. Subscribe to the 'can' socket to receive raw CAN data
    can_sock = messaging.sub_sock('can', timeout=100)

    print("--- Starting Door Lock Monitor ---")
    print(f"Monitoring '{TARGET_SIGNAL}' from '{MESSAGE_TO_MONITOR}'...")
    print(f"Condition: '{CONDITION_SIGNAL}' must be 0.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            # 3. Receive the latest CAN messages from the socket
            can_msgs = messaging.drain_sock_raw(can_sock)
            
            # 4. Feed the raw messages into the parser to get meaningful signal values
            can_parser.update_strings(can_msgs)

            # Clear the terminal for a clean, refreshing display
            os.system('clear' if os.name == 'posix' else 'cls')
            print("--- Door Lock Monitor Status ---")

            # 5. Check our condition and display the target signal's value
            # We access the parsed values from the `vl` (values) dictionary of the parser
            if can_parser.vl[MESSAGE_TO_MONITOR][CONDITION_SIGNAL] == 0:
                # If the condition is met, get and display the value of UNKNOWN_183
                unknown_183_status = can_parser.vl[MESSAGE_TO_MONITOR][TARGET_SIGNAL]
                print(f"\n?? Condition MET ({CONDITION_SIGNAL} == 0)")
                print(f"\n   -> Status of {TARGET_SIGNAL}: {unknown_183_status}")
            else:
                # If the condition is not met, show a waiting message
                print(f"\n?? Waiting... ({CONDITION_SIGNAL} is not 0)")
            
            print(f"\nLast update: {time.strftime('%H:%M:%S')}")

            # 6. Wait for the specified delay before the next refresh
            time.sleep(REFRESH_DELAY)

    except KeyboardInterrupt:
        print("\n\n--- Monitor stopped by user. ---")
    except Exception as e:
        print(f"\nAn error occurred: {e}")

if __name__ == "__main__":
    monitor_signal_status()