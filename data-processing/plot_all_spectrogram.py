import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CSV_FILE = "histogram.csv"
NUM_BINS = 1024
BIN_COLS = [str(i) for i in range(NUM_BINS)]
GRID_ROWS = 4
GRID_COLS = 2

def load_histogram_data(csv_path):
    df = pd.read_csv(csv_path)
    return df

def plot_histogram_spectrograms(df):
    fig, axes = plt.subplots(GRID_ROWS, GRID_COLS, figsize=(16, 10), constrained_layout=True)
    axes = axes.flatten()

    cam_ids = sorted(df['cam_id'].unique())

    for i, cam_id in enumerate(cam_ids):
        ax = axes[i]
        cam_df = df[df['cam_id'] == cam_id].sort_values("frame_id")

        # Extract just the bin data into a 2D array: rows = frames, cols = bins
        histo_matrix = cam_df[BIN_COLS].to_numpy()

        # Plot as spectrogram-like image
        im = ax.imshow(
            histo_matrix,
            aspect='auto',
            interpolation='nearest',
            origin='lower',
            cmap='viridis'
        )
        ax.set_title(f"Camera {cam_id}")
        ax.set_xlabel("Bin")
        ax.set_ylabel("Frame")

    # Hide any unused subplots
    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])

    fig.colorbar(im, ax=axes.tolist(), shrink=0.6, label='Bin Count')
    plt.suptitle("Histogram Spectrograms by Camera", fontsize=16)
    plt.show()

if __name__ == "__main__":
    df = load_histogram_data(CSV_FILE)
    plot_histogram_spectrograms(df)
