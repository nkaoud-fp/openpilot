import requests
import os
import base64

# --- ❗ CONFIGURATION ---

# 1. Your GitHub username and the repository name
GITHUB_USER = "nkaoud-fp"
REPO_NAME = "openpilot"

# 2. The branch you want to push your custom files to
CUSTOM_BRANCH = "auto"

# 3. The commit message to use
COMMIT_MESSAGE = "feat: Update custom files via API"

# 4. List your specific files to push
#    Format: {"C:\\path\\to\\your\\local\\file.py": "path/inside/repo/file.py"}
CUSTOM_FILES = {
    "C:\\commaai\\custom_files\\common\\params.cc": "common/params.cc",
    "C:\\commaai\\custom_files\\frogpilot\\common\\frogpilot_utilities.py": "frogpilot/common/frogpilot_utilities.py",
    "C:\\commaai\\custom_files\\frogpilot\\common\\frogpilot_variables.py": "frogpilot/common/frogpilot_variables.py",
    "C:\\commaai\\custom_files\\frogpilot\\controls\\frogpilot_card.py": "frogpilot/controls/frogpilot_card.py",
    "C:\\commaai\\custom_files\\frogpilot\\controls\\lib\\frogpilot_events.py": "frogpilot/controls/lib/frogpilot_events.py",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\frogpilot_ui.cc": "frogpilot/ui/frogpilot_ui.cc",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\frogpilot_ui.h": "frogpilot/ui/frogpilot_ui.h",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\qt\\offroad\\longitudinal_settings.cc": "frogpilot/ui/qt/offroad/longitudinal_settings.cc",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\qt\\offroad\\longitudinal_settings.h": "frogpilot/ui/qt/offroad/longitudinal_settings.h",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\qt\\offroad\\vehicle_settings.cc": "frogpilot/ui/qt/offroad/vehicle_settings.cc",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\qt\\offroad\\vehicle_settings.h": "frogpilot/ui/qt/offroad/vehicle_settings.h",
    "C:\\commaai\\custom_files\\frogpilot\\ui\\qt\\offroad\\visual_settings.h": "frogpilot/ui/qt/offroad/visual_settings.h",
    "C:\\commaai\\custom_files\\frogpilot\\assets\\other_images\\frogpilot_boot_logo.png": "frogpilot/assets/other_images/frogpilot_boot_logo.png",
    "C:\\commaai\\custom_files\\frogpilot\\assets\\stock_theme\\distance_icons\\auto.png": "frogpilot/assets/stock_theme/distance_icons/auto.png",
    "C:\\commaai\\custom_files\\selfdrive\\assets\\img_spinner_comma.png": "selfdrive/assets/img_spinner_comma.png",
    "C:\\commaai\\custom_files\\selfdrive\\assets\\img_spinner_track.png": "selfdrive/assets/img_spinner_track.png",
    "C:\\commaai\\custom_files\\selfdrive\\car\\toyota\\carcontroller.py": "selfdrive/car/toyota/carcontroller.py",
    "C:\\commaai\\custom_files\\selfdrive\\controls\\controlsd.py": "selfdrive/controls/controlsd.py",
    "C:\\commaai\\custom_files\\selfdrive\\ui\\qt\\onroad\\annotated_camera.cc": "selfdrive/ui/qt/onroad/annotated_camera.cc",
    "C:\\commaai\\custom_files\\selfdrive\\ui\\qt\\onroad\\annotated_camera.h": "selfdrive/ui/qt/onroad/annotated_camera.h",
    "C:\\commaai\\custom_files\\selfdrive\\ui\\qt\\onroad\\onroad_home.cc": "selfdrive/ui/qt/onroad/onroad_home.cc",
    "C:\\commaai\\custom_files\\selfdrive\\ui\\qt\\onroad\\onroad_home.h": "selfdrive/ui/qt/onroad/onroad_home.h",

    # Add any other files you want to update here
}

# --- SCRIPT LOGIC ---

API_URL = f"https://api.github.com/repos/{GITHUB_USER}/{REPO_NAME}"

def main():
    """Main workflow to update files using the GitHub API."""
    # 1. Get GitHub token for authentication
    github_token = os.getenv("GITHUB_TOKEN")
    if not github_token:
        print("❌ Error: GITHUB_TOKEN environment variable not set.")
        return

    headers = {
        "Authorization": f"token {github_token}",
        "Accept": "application/vnd.github.v3+json",
    }

    try:
        # 2. Get the latest commit SHA and tree SHA of the branch
        print(f"Fetching details for branch '{CUSTOM_BRANCH}'...")
        branch_url = f"{API_URL}/branches/{CUSTOM_BRANCH}"
        res = requests.get(branch_url, headers=headers)
        res.raise_for_status()
        branch_data = res.json()
        latest_commit_sha = branch_data['commit']['sha']
        base_tree_sha = branch_data['commit']['commit']['tree']['sha']
        print(f"  - Latest commit SHA: {latest_commit_sha[:7]}")

        # 3. Create a blob for each file to be updated
        blobs = []
        for local_path, repo_path in CUSTOM_FILES.items():
            if not os.path.exists(local_path):
                print(f"  - ⚠️  Warning: Source file not found, skipping: {local_path}")
                continue

            print(f"  - Reading and creating blob for '{repo_path}'...")
            with open(local_path, "rb") as f:
                content = f.read()
            
            # For text files, use 'utf-8'. For images/binary, keep as is.
            # Base64 encoding is required by the API.
            encoded_content = base64.b64encode(content).decode('utf-8')

            blob_url = f"{API_URL}/git/blobs"
            blob_payload = {"content": encoded_content, "encoding": "base64"}
            res = requests.post(blob_url, headers=headers, json=blob_payload)
            res.raise_for_status()
            blob_sha = res.json()['sha']
            
            blobs.append({
                "path": repo_path.replace("\\", "/"), # Ensure forward slashes
                "mode": "100644", # File mode
                "type": "blob",
                "sha": blob_sha,
            })

        if not blobs:
            print("No valid files to update. Exiting.")
            return

        # 4. Create a new tree with the file blobs
        print("\nCreating a new tree with updated file references...")
        tree_url = f"{API_URL}/git/trees"
        tree_payload = {"base_tree": base_tree_sha, "tree": blobs}
        res = requests.post(tree_url, headers=headers, json=tree_payload)
        res.raise_for_status()
        new_tree_sha = res.json()['sha']
        print(f"  - New tree SHA: {new_tree_sha[:7]}")

        # 5. Create a new commit pointing to the new tree
        print("\nCreating a new commit...")
        commit_url = f"{API_URL}/git/commits"
        commit_payload = {
            "message": COMMIT_MESSAGE,
            "tree": new_tree_sha,
            "parents": [latest_commit_sha],
        }
        res = requests.post(commit_url, headers=headers, json=commit_payload)
        res.raise_for_status()
        new_commit_sha = res.json()['sha']
        print(f"  - New commit SHA: {new_commit_sha[:7]}")

        # 6. Update the branch reference to point to the new commit
        print(f"\nUpdating branch '{CUSTOM_BRANCH}' to point to the new commit...")
        ref_url = f"{API_URL}/git/refs/heads/{CUSTOM_BRANCH}"
        ref_payload = {"sha": new_commit_sha}
        res = requests.patch(ref_url, headers=headers, json=ref_payload)
        res.raise_for_status()

        print("\n🎉 Process complete! Your files have been pushed to GitHub.")

    except requests.exceptions.RequestException as e:
        print(f"\n❌ An API error occurred: {e}")
        if e.response is not None:
            print(f"--- Response ---\n{e.response.json()}")

if __name__ == "__main__":
    main()