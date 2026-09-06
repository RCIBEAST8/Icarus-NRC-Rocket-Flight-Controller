import glob
import sys
import numpy as np

OUTPUT_FILE = "NRCMINcd_RANGE_data.dat"
TARGET_POINTS = 200
TRANSIENT_CUTOFF_RATIO = 0.40

dat_files = glob.glob("postProcessing/forceCoeffs/**/forceCoeffs.dat", recursive=True)

if not dat_files:
    print("Error: forceCoeffs.dat not found.")
    sys.exit(1)

data = np.loadtxt(dat_files[0], comments='#')

if data.ndim != 2 or len(data) == 0:
    print("Error: File is empty or malformed.")
    sys.exit(1)

start_index = int(len(data) * TRANSIENT_CUTOFF_RATIO)
steady_data = data[start_index:]

if len(steady_data) == 0:
    print("Error: Not enough data points past transient threshold.")
    sys.exit(1)

if len(steady_data) > TARGET_POINTS:
    sample_indices = np.linspace(0, len(steady_data) - 1, TARGET_POINTS, dtype=int)
    sampled_data = steady_data[sample_indices]
else:
    sampled_data = steady_data

time_vals = sampled_data[:, 0]
cd_vals = sampled_data[:, 2]

with open(OUTPUT_FILE, "w") as f:
    f.write("Time\tCd\n")
    for t, cd in zip(time_vals, cd_vals):
        f.write(f"{t:.6f}\t{cd:.6f}\n")

print(f"Finished Successfully")
