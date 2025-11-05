#!/usr/bin/env python3
import time
from collections import defaultdict

try:
    import cereal.messaging as messaging
except ImportError:
    print("Error: cereal library not found. Make sure you are running this on a comma device.")
    exit()

CAPTURE_DURATION = 5  # seconds

def capture_can_data(duration):
    """Captures CAN data for a set duration and organizes it by message ID."""
    print(f"Capturing data for {duration} seconds...")
    can_sock = messaging.sub_sock('can')
    
    # Give a moment for the socket to connect
    time.sleep(0.5) 
    
    captured_data = defaultdict(list)
    start_time = time.monotonic()
    
    spinner = ['|', '/', '-', '\\']
    i = 0
    
    while time.monotonic() - start_time < duration:
        print(f"\r[{spinner[i % len(spinner)]}] Capturing... ", end="")
        i += 1
        
        can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
        for msg in can_msgs:
            for can_msg in msg.can:
                # Store the raw payload (bytes)
                captured_data[can_msg.address].append(can_msg.dat)
        time.sleep(0.01)

    print("\r✅ Capture complete.     ")
    return captured_data

def analyze_and_print_results(baseline_data, event_data):
    """Compares the two datasets and prints a clear report of the differences."""
    print("\n" + "="*50)
    print("           CAN Message Difference Report")
    print("="*50)

    baseline_ids = set(baseline_data.keys())
    event_ids = set(event_data.keys())

    # 1. Find messages that ONLY appear during the event
    new_ids = event_ids - baseline_ids
    if new_ids:
        print("\n--- 🆕 New Messages (Appeared During Event) ---")
        for msg_id in sorted(list(new_ids)):
            # Get unique payloads and convert to hex for printing
            unique_payloads = {p.hex() for p in event_data[msg_id]}
            print(f"  ID: {msg_id} (0x{msg_id:X})")
            print(f"     - Payloads: {list(unique_payloads)}")
    else:
        print("\n--- 🆕 No new messages appeared during the event. ---")
        
    # 2. Find messages that DISAPPEAR during the event
    disappeared_ids = baseline_ids - event_ids
    if disappeared_ids:
        print("\n--- 👻 Disappeared Messages (Vanished During Event) ---")
        for msg_id in sorted(list(disappeared_ids)):
            unique_payloads = {p.hex() for p in baseline_data[msg_id]}
            print(f"  ID: {msg_id} (0x{msg_id:X})")
            print(f"     - Last seen payloads: {list(unique_payloads)}")
    else:
        print("\n--- 👻 No messages disappeared during the event. ---")

    # 3. Find messages present in both, but whose data changed
    common_ids = baseline_ids.intersection(event_ids)
    changed_messages = []
    for msg_id in common_ids:
        baseline_payloads = set(p.hex() for p in baseline_data[msg_id])
        event_payloads = set(p.hex() for p in event_data[msg_id])
        
        if baseline_payloads != event_payloads:
            changed_messages.append({
                'id': msg_id,
                'baseline': list(baseline_payloads),
                'event': list(event_payloads)
            })
            
    if changed_messages:
        print("\n--- 🔄 Changed Messages (Data Payload Altered) ---")
        for msg in sorted(changed_messages, key=lambda x: x['id']):
            print(f"  ID: {msg['id']} (0x{msg['id']:X})")
            print(f"     - Baseline: {msg['baseline']}")
            print(f"     - Event:    {msg['event']}")
    else:
        print("\n--- 🔄 No existing messages changed their data payload. ---")
        
    print("\n" + "="*50)
    print("Report complete. The most likely candidates are in")
    print("the 'New Messages' or 'Changed Messages' sections.")
    print("="*50)


def main():
    """Main function to guide the user through the capture and analysis process."""
    print("--- CAN State Change Finder ---")
    print("This script will help you find a CAN message related to a specific event.")
    
    # --- Baseline Capture ---
    print("\n[STEP 1 of 2]")
    input("Please ensure the car is in its NORMAL state (e.g., key fob is present).\nPress Enter to begin capturing the baseline data...")
    baseline_data = capture_can_data(CAPTURE_DURATION)
    
    # --- Event Capture ---
    print("\n[STEP 2 of 2]")
    input(f"Now, please trigger the EVENT (e.g., move the key fob away until '{'KeyFob unavailable'}' appears).\nOnce the warning is stable on your dashboard, press Enter to capture the event data...")
    event_data = capture_can_data(CAPTURE_DURATION)
    
    # --- Analysis ---
    analyze_and_print_results(baseline_data, event_data)

if __name__ == "__main__":
    main()