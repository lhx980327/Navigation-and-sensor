import numpy as np
import matplotlib.pyplot as plt
import allantools
import pandas as pd

imu_data = pd.read_csv("/home/haoxinliu/catkin_ws/src/eece5554/LAB3/src/imu_driver/data/imu_data_5.csv")
quat_x = imu_data['field.orientation.x'].to_numpy()
quat_y = imu_data['field.orientation.y'].to_numpy()
quat_z = imu_data['field.orientation.z'].to_numpy()

# Extract magnetic field data
magX = imu_data['field.magnetic_field.x'].to_numpy()
magY = imu_data['field.magnetic_field.y'].to_numpy()
magZ = imu_data['field.magnetic_field.z'].to_numpy()
gyroX = imu_data['field.angular_velocity.x'].to_numpy()
gyroY = imu_data['field.angular_velocity.y'].to_numpy()
gyroZ = imu_data['field.angular_velocity.z'].to_numpy()

accX = imu_data['field.linear_acceleration.x'].to_numpy()
accY = imu_data['field.linear_acceleration.y'].to_numpy()
accZ = imu_data['field.linear_acceleration.z'].to_numpy()

data_lists = [gyroX, gyroY, gyroZ, accX, accY, accZ, quat_x, quat_y, quat_z, magX, magY, magZ]
labels = ["GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "QuatX", "QuatY", "QuatZ", "MagX", "MagY", "MagZ"]
t0 = 1/40  # for 40 hz
# Compute mean and standard deviation for each field and print the results
fields = [
          'field.angular_velocity.x', 'field.angular_velocity.y', 'field.angular_velocity.z', 
          'field.linear_acceleration.x', 'field.linear_acceleration.y', 'field.linear_acceleration.z', 
          'field.magnetic_field.x', 'field.magnetic_field.y', 'field.magnetic_field.z',
          'field.orientation.x', 'field.orientation.y', 'field.orientation.z']

for field in fields:
    mean_value = imu_data[field].mean()
    std_value = imu_data[field].std()
    print(f"Mean for {field}: {mean_value:.4f}, Standard Deviation for {field}: {std_value:.4f}")
for idx, data in enumerate(data_lists):
    tau, adev, _, _ = allantools.adev(data)

    # Plotting
    plt.figure(figsize=(10, 8))
    plt.loglog(tau, adev, label=f'{labels[idx]} ADEV')
    plt.title(f'Allan Deviation for {labels[idx]}')
    plt.xlabel(r'$\tau$ (s)')
    plt.ylabel(r'$\sigma(\tau)$')
    plt.grid(True, which="both", ls="--")
    plt.legend()
    plt.show()
for idx, data in enumerate(data_lists):
    tau, adev, _, _ = allantools.adev(data)
    logadev = np.log10(adev)
    logtau = np.log10(tau)
    dlogadev = np.diff(logadev) / np.diff(logtau)

    # Angle Random Walk (N)
    slope_n = -0.5
    idx_n = np.argmin(np.abs(dlogadev - slope_n))
    N = 10 ** (logadev[idx_n] - slope_n * logtau[idx_n])

    # Rate Random Walk (K)
    slope_k = 0.5
    idx_k = np.argmin(np.abs(dlogadev - slope_k))
    K = 10 ** (logadev[idx_k] - slope_k * np.log10(3))

    # Bias Instability (B)
    slope_b = 0
    idx_b = np.argmin(np.abs(dlogadev - slope_b))
    scfB = np.sqrt(2 * np.log(2) / np.pi)
    B = 10 ** (logadev[idx_b] - np.log10(scfB))

    # Plotting
    plt.figure(figsize=(10, 8))
    plt.loglog(tau, adev, label=f'{labels[idx]} ADEV')
    plt.loglog(tau, N / np.sqrt(tau), '--', label='N')
    plt.loglog(tau, K * np.sqrt(tau/3), '--', label='K')
    plt.loglog(tau, np.full_like(tau, B * scfB), '--', label=f'0.664B = {B:.2e}')
    plt.axvline(tau[idx_n], color='red', linestyle='--', linewidth=0.5)
    plt.axvline(tau[idx_k], color='blue', linestyle='--', linewidth=0.5)
    plt.axvline(tau[idx_b], color='green', linestyle='--', linewidth=0.5)
    plt.title(f'Allan Deviation for {labels[idx]}')
    plt.xlabel(r'$\tau$ (s)')
    plt.ylabel(r'$\sigma(\tau)$')
    plt.grid(True, which="both", ls="--")
    plt.legend()
    plt.show()
    print(f"For {labels[idx]}: N = {N:.4e}, K = {K:.4e}, B = {B:.4e}")

