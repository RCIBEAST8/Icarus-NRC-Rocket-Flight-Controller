import glob
import time
import numpy as np

OUTPUT_FILE = "rangeCD.dat"
INTERVAL_SECONDS = 1800

print(f"cd processing started")
try:
    while True:
        dat_files = glob.glob("postProcessing/forceCoeffs/**/forceCoeffs.dat", recursive=True)

        if not dat_files:
            print("STOP, forceCoeffs.dat not found")
        else:
            data = np.loadtxt(dat_files[0], comments='#')

            if data.ndim == 2 and len(data) > 0:
                sim_time = data[:, 0]
                cd = data[:, 2]

                half_time = sim_time[-1] / 2
                mask = sim_time >= half_time
                steady_time = sim_time[mask]
                steady_cd = cd[mask]

                mean_cd = np.mean(steady_cd)
                std_cd = np.std(steady_cd)
                min_cd = np.min(steady_cd)
                max_cd = np.max(steady_cd)
                final_cd = cd[-1]
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

                print(f"[{timestamp}]")
                print(f"  Steady-State Mean C_d: {mean_cd:.5f}")
                print(f"  Standard Deviation:   ±{std_cd:.5f}")
                print(f"  Saved to: {OUTPUT_FILE}")
                print("-" * 40)

                with open(OUTPUT_FILE, "w") as f:
                    f.write("# OpenFOAM Drag Coefficient Summary Data\n")
                    f.write(f"# Generated: {timestamp}\n")
                    f.write(f"# Source File: {dat_files[0]}\n")
                    f.write("# ----------------------------------------\n")
                    f.write("Metric\tValue\tUnit\n")
                    f.write(f"Mean_Cd\t{mean_cd:.6f}\t-\n")
                    f.write(f"Std_Dev\t{std_cd:.6f}\t-\n")
                    f.write(f"Min_Cd\t{min_cd:.6f}\t-\n")
                    f.write(f"Max_Cd\t{max_cd:.6f}\t-\n")
                    f.write(f"Final_Cd\t{final_cd:.6f}\t-\n")
                    f.write(f"Evaluated_Start_Time\t{steady_time[0]:.4f}\ts\n")
                    f.write(f"Evaluated_End_Time\t{steady_time[-1]:.4f}\ts\n")

            else:
                print("failed ")

        time.sleep(INTERVAL_SECONDS)

except KeyboardInterrupt:
    print("\nMonitoring stopped")
