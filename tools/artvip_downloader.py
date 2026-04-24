
# Dataset：https://huggingface.co/datasets/X-Humanoid/ArtVIP

import os
from huggingface_hub import snapshot_download

def download_artvip(target_folder="Scenes/kitchen", output_dir="./ArtVIP_Data"):
    """
    Universal downloader for the ArtVIP dataset.
    :param target_folder: Internal path within the dataset (e.g., 'Scenes/kitchen')
    :param output_dir: Local directory where files will be saved
    """
    repo_id = "X-Humanoid/ArtVIP"
    
    # Define the pattern to include only specific sub-directories or files
    # Using wildcards (*) ensures all nested assets (USDs, textures) are captured
    allow_patterns = [f"{target_folder.strip('/')}/*"]
    
    print(f"🚀 Initializing download for ArtVIP project: {target_folder}")
    print(f"📁 Local destination: {os.path.abspath(output_dir)}")
    print("-" * 50)

    try:
        # snapshot_download is the most stable method for LFS-heavy datasets like ArtVIP
        snapshot_download(
            repo_id=repo_id,
            repo_type="dataset",
            local_dir=output_dir,
            allow_patterns=allow_patterns,
            # Windows compatibility: Disabling symlinks prevents permission errors 
            # and ensures absolute paths work correctly in Isaac Sim
            local_dir_use_symlinks=False,  
            # Enable resumable downloads in case of network instability
            resume_download=True,          
            # Use multiple workers to speed up the download of small texture files
            max_workers=4                  
        )
        print("-" * 50)
        print(f"✅ Download successful! Files located at: {output_dir}")
    except Exception as e:
        print(f"❌ Download failed: {str(e)}")

if __name__ == "__main__":
    print("=== ArtVIP Universal Downloader ===")
    print("Common path examples:")
    print("1. Scenes/kitchen                         (Full kitchen scenes)")
    print("2. Articulated_objects/small_furniture/* (All small furniture items)")
    print("3. Articulated_objects/appliances         (Home appliances)")
    
    # User input for the specific asset category needed for simulation
    folder = input("\nEnter the dataset path keyword to download: ").strip()
    
    # User input for the local save path (defaults to ArtVIP_Download)
    out = input("Enter local directory name (Default: ArtVIP_Download): ").strip() or "ArtVIP_Download"
    
    download_artvip(folder, out)