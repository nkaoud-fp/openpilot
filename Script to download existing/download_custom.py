import os
import requests

# --- Configuration ---
GIT_USER = "nkaoud-fp"
REPO = "openpilot"
BRANCH = "auto"

FILES_TO_DOWNLOAD = [
    "common/params.cc",
    "frogpilot/common/frogpilot_utilities.py",
    "frogpilot/common/frogpilot_variables.py",
    "frogpilot/controls/frogpilot_card.py",
    "frogpilot/controls/lib/frogpilot_events.py",
    "frogpilot/ui/frogpilot_ui.cc",
    "frogpilot/ui/frogpilot_ui.h",
    "frogpilot/ui/qt/offroad/longitudinal_settings.cc",
    "frogpilot/ui/qt/offroad/longitudinal_settings.h",
    "frogpilot/ui/qt/offroad/vehicle_settings.cc",
    "frogpilot/ui/qt/offroad/vehicle_settings.h",
    "frogpilot/ui/qt/offroad/visual_settings.h",
    "selfdrive/assets/img_spinner_comma.png",
    "selfdrive/assets/img_spinner_track.png",
    "selfdrive/car/toyota/carcontroller.py",
    "selfdrive/controls/controlsd.py",
    "selfdrive/ui/qt/onroad/annotated_camera.cc",
    "selfdrive/ui/qt/onroad/annotated_camera.h",
    "selfdrive/ui/qt/onroad/onroad_home.cc",
    "frogpilot/assets/other_images/frogpilot_boot_logo.png",
    "frogpilot/assets/stock_theme/distance_icons/auto.png",
    "selfdrive/ui/qt/onroad/onroad_home.h",
]

# --- Main Script ---

def download_file(file_path):
    """
    Downloads a single file from the GitHub repository, creating local
    directories as needed.
    """
    # Construct the full URL to the raw file
    base_url = f"https://raw.githubusercontent.com/{GIT_USER}/{REPO}/{BRANCH}"
    url = f"{base_url}/{file_path}"

    try:
        # Get the directory part of the file path (e.g., "common/")
        local_dir = os.path.dirname(file_path)

        # Create the local directory structure if it doesn't exist
        if local_dir and not os.path.exists(local_dir):
            print(f"📁 Creating directory: {local_dir}")
            os.makedirs(local_dir)

        # Send a request to download the file
        print(f"⏳ Downloading {file_path}...")
        response = requests.get(url, timeout=15)
        
        # Raise an error for bad status codes (like 404 Not Found)
        response.raise_for_status()

        # Write the content to the local file
        # 'wb' mode is used to handle all file types (text, images, etc.)
        with open(file_path, 'wb') as f:
            f.write(response.content)
            
        print(f"✅ Success: Saved {file_path}")

    except requests.exceptions.RequestException as e:
        print(f"❌ Error downloading {file_path}: {e}")

def main():
    """
    Main function to loop through the file list and download each one.
    """
    print(f"🚀 Starting download from '{GIT_USER}/{REPO}' on branch '{BRANCH}'...\n")
    
    for file in FILES_TO_DOWNLOAD:
        download_file(file)
        print("-" * 20) # Separator for clarity
        
    print("\n🎉 Download process finished!")

if __name__ == "__main__":
    main()