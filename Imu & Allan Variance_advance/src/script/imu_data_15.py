import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from bagpy import bagreader
import math
# Step 1: Extract data from the ROS bag and save it to a CSV

# Step 2: Visualizing the IMU data from the CSV
imu_data = pd.read_csv("E:/vscode/LAB4/src/data/imu_data_15.csv")  
time_stamps = imu_data['%time']
def quaternion_to_euler(q0, q1, q2, q3):
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    
    return roll, pitch, yaw
euler_angles = imu_data.apply(lambda row: quaternion_to_euler(row['field.orientation.w'], 
                                                              row['field.orientation.x'], 
                                                              row['field.orientation.y'], 
                                                              row['field.orientation.z']), axis=1)

# Splitting the Euler angles into separate columns
imu_data['euler_roll'], imu_data['euler_pitch'], imu_data['euler_yaw'] = zip(*euler_angles)
plt.figure(figsize=(10, 8))

euler_angles_list = ['euler_roll', 'euler_pitch', 'euler_yaw']
for idx, angle in enumerate(euler_angles_list):
    plt.plot(time_stamps.to_numpy(), imu_data[angle].to_numpy(), label=f'{["Roll", "Pitch", "Yaw"][idx]}')

plt.xlabel('Time (s)')
plt.ylabel('Euler Angle (rad)')
plt.legend(loc='upper right')
plt.title("Euler Angles (Roll, Pitch, Yaw) vs Time")
plt.grid(True)
plt.tight_layout()
plt.show()
# Calculate the mean for each of the Euler angles
mean_roll = imu_data['euler_roll'].mean()
mean_pitch = imu_data['euler_pitch'].mean()
mean_yaw = imu_data['euler_yaw'].mean()

# Calculate the error for each of the Euler angles
roll_error = imu_data['euler_roll'] - mean_roll
pitch_error = imu_data['euler_pitch'] - mean_pitch
yaw_error = imu_data['euler_yaw'] - mean_yaw

# Plot the histograms for the errors
plt.figure(figsize=(10, 8))
plt.hist(roll_error, bins=50, alpha=0.5, label='Roll Error', color='r')
plt.hist(pitch_error, bins=50, alpha=0.5, label='Pitch Error', color='g')
plt.hist(yaw_error, bins=50, alpha=0.5, label='Yaw Error', color='b')
plt.xlabel('Error Value')
plt.ylabel('Frequency')
plt.title('Histogram of Euler Angle Errors (Roll, Pitch, Yaw)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


""" print('mean for angular')
print( imu_data['field.angular_velocity.x'].mean(), imu_data['field.angular_velocity.y'].mean(), imu_data['field.angular_velocity.z'].mean())
print('mean for orientation')
print(imu_data['field.orientation.x'].mean(), imu_data['field.orientation.y'].mean(), imu_data['field.orientation.z'].mean())
print('mean for linear')
print(imu_data['field.linear_acceleration.x'].mean(),imu_data['field.linear_acceleration.y'].mean(),imu_data['field.linear_acceleration.z'].mean())
print('mean for magnetic')
print(imu_data['field.magnetic_field.x'].mean(),imu_data['field.magnetic_field.y'].mean(),imu_data['field.magnetic_field.z'].mean())
# Assuming the IMU data has columns 'time', 'orientationX', 'orientationY', and 'orientationZ'
time_stamps = imu_data['%time']  
orientations = [imu_data['field.orientation.x'], imu_data['field.orientation.y'], imu_data['field.orientation.z']]
angular_velocities = [
            imu_data['field.angular_velocity.x'], 
            imu_data['field.angular_velocity.y'], 
            imu_data['field.angular_velocity.z']
        ]

plt.figure(figsize=(10, 8))
for idx, orientation in enumerate(orientations):
    plt.plot(time_stamps.to_numpy(), orientation.to_numpy(), label=f'Orientation {["X", "Y", "Z"][idx]}')


plt.xlabel('Time (s)')
plt.ylabel('Orientation (rad)')
plt.legend(loc='east')
plt.title("Orientation vs Time")
plt.grid(True)
plt.tight_layout()
plt.show()



# Plotting angular velocities vs time on a single graph
plt.figure(figsize=(10, 8))
        
# Plot each angular velocity component
for idx, velocity in enumerate(angular_velocities):
            plt.plot(time_stamps.to_numpy(), velocity.to_numpy(), label=f'Angular Velocity {["X", "Y", "Z"][idx]}')

plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend(loc='upper right')
plt.title("Angular Velocities (X, Y, Z) vs Time")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 8))
 
# Directly extracting and plotting the components
plt.plot(time_stamps.to_numpy(), imu_data['field.linear_acceleration.x'].to_numpy(), label='Linear Acceleration X')
plt.plot(time_stamps.to_numpy(), imu_data['field.linear_acceleration.y'].to_numpy(), label='Linear Acceleration Y')
plt.plot(time_stamps.to_numpy(), imu_data['field.linear_acceleration.z'].to_numpy(), label='Linear Acceleration Z')

plt.xlabel('Time (s)')
plt.ylabel('Linear Acceleration (m/s^2)')
plt.legend(loc='upper right')
plt.title("Linear Accelerations (X, Y, Z) vs Time")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 8))

# Directly extracting and plotting the components
plt.plot(time_stamps.to_numpy(), imu_data['field.magnetic_field.x'].to_numpy(), label='Magnetic Field X')
plt.plot(time_stamps.to_numpy(), imu_data['field.magnetic_field.y'].to_numpy(), label='Magnetic Field Y')
plt.plot(time_stamps.to_numpy(), imu_data['field.magnetic_field.z'].to_numpy(), label='Magnetic Field Z')

plt.xlabel('Time (s)')
plt.ylabel('Magnetic Field (Tesla)')
plt.legend(loc='upper right')
plt.title("Magnetic Field Components (X, Y, Z) vs Time")
plt.grid(True)
plt.tight_layout()
plt.show() """

angular_velocity_x_error = imu_data['field.angular_velocity.x'] - imu_data['field.angular_velocity.x'].mean()
angular_velocity_y_error = imu_data['field.angular_velocity.y'] - imu_data['field.angular_velocity.y'].mean()
angular_velocity_z_error = imu_data['field.angular_velocity.z'] - imu_data['field.angular_velocity.z'].mean()
magnetic_field_x_error = imu_data['field.magnetic_field.x'] - imu_data['field.magnetic_field.x'].mean()
magnetic_field_y_error = imu_data['field.magnetic_field.y'] - imu_data['field.magnetic_field.y'].mean()
magnetic_field_z_error = imu_data['field.magnetic_field.z'] - imu_data['field.magnetic_field.z'].mean()
linear_accel_x_error = imu_data['field.linear_acceleration.x'] - imu_data['field.linear_acceleration.x'].mean()
linear_accel_y_error = imu_data['field.linear_acceleration.y'] - imu_data['field.linear_acceleration.y'].mean()
linear_accel_z_error = imu_data['field.linear_acceleration.z'] - imu_data['field.linear_acceleration.z'].mean()
orientation_x_error = imu_data['field.orientation.x'] - imu_data['field.orientation.x'].mean()
orientation_y_error = imu_data['field.orientation.y'] - imu_data['field.orientation.y'].mean()
orientation_z_error = imu_data['field.orientation.z'] - imu_data['field.orientation.z'].mean()
""" plt.figure(figsize=(10, 8))
plt.hist(orientation_x_error, bins=50, alpha=0.5, label='Orientation X Error', color='r')
plt.hist(orientation_y_error, bins=50, alpha=0.5, label='Orientation Y Error', color='g')
plt.hist(orientation_z_error, bins=50, alpha=0.5, label='Orientation Z Error', color='b')
plt.xlabel('Error Value')
plt.ylabel('Frequency')
plt.title('Histogram of Orientation Errors (X, Y, Z)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
plt.figure(figsize=(10, 8))
plt.hist(magnetic_field_x_error, bins=50, alpha=0.5, label='Magnetic Field X Error', color='r')
plt.hist(magnetic_field_y_error, bins=50, alpha=0.5, label='Magnetic Field Y Error', color='g')
plt.hist(magnetic_field_z_error, bins=50, alpha=0.5, label='Magnetic Field Z Error', color='b')
plt.xlabel('Error Value')
plt.ylabel('Frequency')
plt.title('Histogram of Magnetic Field Errors (X, Y, Z)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show() """
plt.figure(figsize=(10, 8))
plt.hist(linear_accel_x_error, bins=50, alpha=0.5, label='Linear Acceleration X Error', color='r')
plt.hist(linear_accel_y_error, bins=50, alpha=0.5, label='Linear Acceleration Y Error', color='g')
plt.hist(linear_accel_z_error, bins=50, alpha=0.5, label='Linear Acceleration Z Error', color='b')
plt.xlabel('Error Value')
plt.ylabel('Frequency')
plt.title('Histogram of Linear Acceleration Errors (X, Y, Z)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


""" plt.figure(figsize=(10, 8))

# Plot histogram for angular_velocity.x error
plt.hist(angular_velocity_x_error, bins=50, alpha=0.5, label='Angular Velocity X Error', color='r')

# Plot histogram for angular_velocity.y error
plt.hist(angular_velocity_y_error, bins=50, alpha=0.5, label='Angular Velocity Y Error', color='g')

# Plot histogram for angular_velocity.z error
plt.hist(angular_velocity_z_error, bins=50, alpha=0.5, label='Angular Velocity Z Error', color='b')

plt.xlabel('Error Value')
plt.ylabel('Frequency')
plt.title('Histogram of Angular Velocity Errors (X, Y, Z)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
 """