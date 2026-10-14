import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

CSV_FILE      = "histogram.csv"
CAMERA_ID     = 0
NUM_BINS      = 1024
BIN_COLS = [str(i) for i in range(NUM_BINS)]
FRAME_ID_MAX  = 256            # frame_id wraps 255 → 0


def plot_weighted_average(csv_path, cam_id):
    df = pd.read_csv(csv_path)

    # ---- Select camera ----
    cam_df = df[df['cam_id'] == cam_id]
    if cam_df.empty:
        print(f"No data found for camera {cam_id}")
        return

    # ---- Build monotonic frame index (handles rollover at 255 → 0) ----
    logical_indices = []
    last_frame_id   = None
    rollover_count  = 0

    for _, row in cam_df.iterrows():
        current = row['frame_id']
        if last_frame_id is not None and current < last_frame_id:
            rollover_count += 1
        logical_indices.append(rollover_count * FRAME_ID_MAX + current)
        last_frame_id = current

    cam_df = cam_df.assign(logical_frame_index=logical_indices) \
                   .sort_values("logical_frame_index")

    # ---- Extract histogram bins ----
    histo_matrix = cam_df[BIN_COLS].to_numpy()
    histo_matrix[:, -1] = 0                       # zero-out bin 1023

    frame_ids = cam_df['logical_frame_index'].to_numpy()
    bin_idx   = np.arange(NUM_BINS)

    # ---- Weighted average (μ) ----
    sums          = histo_matrix.sum(axis=1)
    weighted_sums = histo_matrix @ bin_idx
    mu            = np.divide(weighted_sums, sums,
                              out=np.zeros_like(sums, dtype=float),
                              where=sums != 0)

    # ---- Standard deviation (σ) ----
    # E[X²] = Σ(hist_i * bin_i²) / Σ(hist_i)
    second_moment = np.divide(histo_matrix @ (bin_idx ** 2), sums,
                              out=np.zeros_like(sums, dtype=float),
                              where=sums != 0)
    variance      = second_moment - mu ** 2
    sigma         = np.sqrt(np.clip(variance, 0, None))  # avoid negatives

    # ---- Plot ----
    fig, ax1 = plt.subplots(figsize=(10, 5))

    ln_mu,   = ax1.plot(frame_ids, mu,    'o-', markersize=3,
                        label="Weighted avg (μ)")
    ln_sigma, = ax1.plot(frame_ids, sigma, 's--', markersize=3,
                         color='tab:orange', label="Std dev (σ)")

    ax1.set_xlabel("Frame ID")
    ax1.set_ylabel("Bin index (μ, σ)")
    ax1.grid(True)

    # ---- Optional temperature on a twin axis ----
    lines = [ln_mu, ln_sigma]
    labels = [ln.get_label() for ln in lines]

    if 'temperature' in cam_df.columns:
        temperature = cam_df['temperature'].to_numpy()
        ax2 = ax1.twinx()
        ln_temp, = ax2.plot(frame_ids, temperature,
                            'd-.', markersize=3, color='tab:red',
                            label="Temperature (°C)")
        ax2.set_ylabel("Temperature (°C)")
        lines.append(ln_temp)
        labels.append(ln_temp.get_label())

    ax1.legend(lines, labels, loc='best')

    title_extra = " & Temperature" if 'temperature' in cam_df.columns else ""
    plt.title(f"Camera {cam_id} – Weighted Average, σ{title_extra} Over Time")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_weighted_average(CSV_FILE, cam_id=CAMERA_ID)
