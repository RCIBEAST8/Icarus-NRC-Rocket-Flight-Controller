import glob
import numpy as np

dat_files = glob.glob("postProcessing/forceCoeffs/**/forceCoeffs.dat", recursive=True)
if not dat_files:
    print("Error: forceCoeffs.dat not found.")
    exit(1)

data = np.loadtxt(dat_files[0], comments='#')
time, cd = data[:, 0], data[:, 2]

# Discard the first half of the simulation to ignore startup transients
steady_cd = cd[time >= (time[-1] / 2)]

print(f"Final Average C_d: {np.mean(steady_cd):.4f}")
print(f"Standard Deviation: ±{np.std(steady_cd):.4f}")
