import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "histogram.csv"
NUM_BINS = 1024
BIN_COLS = [str(i) for i in range(NUM_BINS)]
CAMERA_ID = 0
FRAME_ID_MAX = 256  # Since frame_id wraps around at 255 -> 0

def plot_camera_histograms(csv_path):
    df = pd.read_csv(csv_path)

    # Filter to specific camera
    cam_df = df[df['cam_id'] == CAMERA_ID].copy()

    if cam_df.empty:
        print(f"No data found for camera {CAMERA_ID}.")
        return

    # Sort by time to preserve actual order
    # cam_df = cam_df.sort_values(by="Time [s]").reset_index(drop=True)

    # Track logical frame index across frame_id rollover
    logical_indices = []
    last_frame_id = None
    rollover_count = 0

    for _, row in cam_df.iterrows():
        current_frame_id = row['frame_id']
        if last_frame_id is not None:
            # Detect wraparound
            if current_frame_id < last_frame_id:
                rollover_count += 1
        logical_index = rollover_count * FRAME_ID_MAX + current_frame_id
        logical_indices.append(logical_index)
        last_frame_id = current_frame_id

    cam_df['logical_frame_index'] = logical_indices

    # Sort by the logical frame index
    cam_df = cam_df.sort_values(by="logical_frame_index")

    # Extract only the histogram bins (columns 2 to 2+1024)
    histo_matrix = cam_df[BIN_COLS].to_numpy()
    histo_matrix[:, -1] = 0  # Optional: zero out last bin for visualization

    # Plot spectrogram
    plt.figure(figsize=(12, 6))
    im = plt.imshow(
        histo_matrix,
        aspect='auto',
        interpolation='nearest',
        origin='lower',
        cmap='viridis'
    )
    plt.colorbar(im, label="Bin Count")
    plt.title(f"Camera {CAMERA_ID} Histogram Spectrogram")
    plt.xlabel("Bin Index")
    plt.ylabel("Frame Index")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_camera_histograms(CSV_FILE)
