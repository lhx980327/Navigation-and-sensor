from os import W_OK
import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
from scipy import integrate
import math

 ########## Read in to csv file, convert to numpy array ##########
bag = bagreader('e:/vscode/LAB4/src/data/data.bag')
imu_data = bag.message_by_topic(topic='/imu')
gps_data = bag.message_by_topic(topic='/gps')
imu_data_csv = pd.read_csv(imu_data)
gps_data_csv = pd.read_csv(gps_data)
imu_data_pd = pd.DataFrame(imu_data_csv, columns=['Time', 'IMU.orientation.x', 'IMU.orientation.y', 'IMU.orientation.z', 'IMU.orientation.w', 'IMU.angular_velocity.x', 'IMU.angular_velocity.y', 'IMU.angular_velocity.z', 'IMU.linear_acceleration.x', 'IMU.linear_acceleration.y', 'IMU.linear_acceleration.z', 'MagField.magnetic_field.x', 'MagField.magnetic_field.y', 'MagField.magnetic_field.z']).astype(float)
gps_data_pd = pd.DataFrame(gps_data_csv, columns=['Time', 'Latitude', 'Longitude', 'Altitude','UTM_Easting', 'UTM_Northing']).astype(float)
imu_data_np = pd.DataFrame.to_numpy(imu_data_pd)
gps_data_np = pd.DataFrame.to_numpy(gps_data_pd)
########## split data in to stationary, circles, moving ##########
# Split IMU data based on provided indices
stationary_imu = imu_data_np[0:403, :]
circles_imu = imu_data_np[403:4804, :]
moving_imu = imu_data_np[4803:, :]

# Find closest corresponding GPS times for each segment of IMU data
gps_stationary_start, = np.where(np.absolute(gps_data_np[:, 0] - imu_data_np[0, 0]) == np.min(np.absolute(gps_data_np[:, 0] - imu_data_np[0, 0])))
gps_stationary_end, = np.where(np.absolute(gps_data_np[:, 0] - imu_data_np[402, 0]) == np.min(np.absolute(gps_data_np[:, 0] - imu_data_np[402, 0])))
gps_circles_start, = np.where(np.absolute(gps_data_np[:, 0] - imu_data_np[403, 0]) == np.min(np.absolute(gps_data_np[:, 0] - imu_data_np[403, 0])))
gps_circles_end, = np.where(np.absolute(gps_data_np[:, 0] - imu_data_np[4802, 0]) == np.min(np.absolute(gps_data_np[:, 0] - imu_data_np[4802, 0])))
gps_moving_start, = np.where(np.absolute(gps_data_np[:, 0] - imu_data_np[4803, 0]) == np.min(np.absolute(gps_data_np[:, 0] - imu_data_np[4803, 0])))

# Extract corresponding segments of GPS data based on the above times
stationary_gps = gps_data_np[gps_stationary_start[0]:gps_stationary_end[0]+1, :]
circles_gps = gps_data_np[gps_circles_start[0]:gps_circles_end[0]+1, :]
moving_gps = gps_data_np[gps_moving_start[0]:, :]

########## plotting data function ##########
def plot_data(x_data, y_data, xlabel, ylabel, title):
    plt.close()
    plt.plot(x_data, y_data)
    plt.grid()
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.show()
########## correcting yaw from corrected mag data ##########
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if np.any(t2 > +1.0) else t2
        t2 = -1.0 if np.any(t2 < -1.0) else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return roll, pitch, yaw # in radians
roll,pitch,yaw = euler_from_quaternion(circles_imu[:, 1], circles_imu[:, 2], circles_imu[:, 3], circles_imu[:, 4])
rollm,pitchm,yawm = euler_from_quaternion(moving_imu[:, 1], moving_imu[:, 2], moving_imu[:, 3], moving_imu[:, 4])
yawm = np.unwrap(yawm)
rollm = np.unwrap(rollm)
pitchm = np.unwrap(pitchm)


def wrap_to_pi(data):
    return np.remainder(data + np.pi, 2*np.pi) - np.pi


 ########## plotting corrected magnetometer data ##########
# plot_data(circles_imu[:, 11], circles_imu[:, 12], 'Mag - X', 'Mag - Y', 'X and Y Magnetometer Data')
circles_imu_avgx = np.mean(circles_imu[:, 11])
circles_imu_avgy = np.mean(circles_imu[:, 12])
moving_imu_avgx = np.mean(moving_imu[:, 11])
moving_imu_avgy = np.mean(moving_imu[:, 12])
corrected_circles_imu_magx = circles_imu[:, 11] - circles_imu_avgx
corrected_circles_imu_magy = circles_imu[:, 12] - circles_imu_avgy
corrected_moving_imu_magx = moving_imu[:, 11] - moving_imu_avgx
corrected_moving_imu_magy = moving_imu[:, 12] - moving_imu_avgy
""" # For circles_imu
corrected_yaw_mag = np.arctan2(corrected_circles_imu_magx, corrected_circles_imu_magy)
corrected_yaw_mag = np.unwrap(corrected_yaw_mag)  # Unwrapping magnetometer readings
corrected_yaw_int = integrate.cumtrapz(circles_imu[:, 7], circles_imu[:, 0], initial=0)
corrected_yaw_int = np.unwrap(corrected_yaw_int)  # Unwrapping after integration """

# For moving_imu
corrected_yaw_magm = np.arctan2(corrected_moving_imu_magx, corrected_moving_imu_magy)-3.0
corrected_yaw_magm = np.unwrap(corrected_yaw_magm)  # Unwrapping magnetometer readings
corrected_yaw_intm = integrate.cumtrapz(moving_imu[:, 7], moving_imu[:, 0], initial=0)
corrected_yaw_intm = np.unwrap(corrected_yaw_intm)  # Unwrapping after integration
yaw_filtered = 0.9 * corrected_yaw_intm+ 0.1* corrected_yaw_magm

yamm = np.unwrap(yawm)
yawmr = [math.radians(angle) for angle in yamm]
plt.figure()
plt.plot(moving_imu[:, 0], yaw_filtered, 'r-', label='Complementary Filter')
plt.plot(moving_imu[:, 0], corrected_yaw_magm, 'g-', label='Mag Corrected Yaw')
plt.plot(moving_imu[:, 0], corrected_yaw_intm, 'k-', label='Angular Integrated Yaw')
plt.grid()
plt.legend()
plt.title('Compared Yaw Data')
plt.ylabel('Yaw (rad)')
plt.xlabel('Time (s)')
plt.show()
# Plot data
plt.figure()
plt.plot(moving_imu[:, 0], yaw_filtered, 'r-', label='Complementary Filter')
plt.plot(moving_imu[:, 0], yawmr, 'b-', label='Raw Yaw')
plt.plot(moving_imu[:, 0], corrected_yaw_magm, 'g-', label='Mag Corrected Yaw')
plt.plot(moving_imu[:, 0], corrected_yaw_intm, 'k-', label='Angular Integrated Yaw')
plt.grid()
plt.legend()
plt.title('Compared Yaw Data')
plt.ylabel('Yaw (rad)')
plt.xlabel('Time (s)')
plt.show()
""" ########## plotting UTM data ##########


plt.plot(gps_data_np[:, 4], gps_data_np[:, 5], 'o-')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('UTM Data')
plt.grid(True)
plt.show()
 """"""
 plt.close()
plt.plot(circles_imu[:, 11], circles_imu[:, 12], 'b-', label='Raw Data')
plt.plot(circles_imu_avgx, circles_imu_avgy, 'ro', label='Raw Data Center')
plt.plot(corrected_circles_imu_magx, corrected_circles_imu_magy, 'g-', label='Hard and Soft Iron Corrected Data')
plt.grid()
plt.legend()
plt.title('X and Y Magnetometer Data')
plt.ylabel('Mag - Y (tesla)')
plt.xlabel('Mag - X (tesla)')
plt.show() """
""" plt.close()
plt.plot(moving_imu[:, 0], corrected_yaw_magm, 'b-', label='Mag Corrected Yaw')
plt.plot(moving_imu[:, 0], corrected_yaw_int_finalm, 'g-', label='Angular Integrated Yaw')
plt.grid()
plt.legend()
plt.title('Corrected Yaw Data')
plt.ylabel('Yaw (rad)')
plt.xlabel('Time (s)')
plt.show() """
 ########## Velocity calculations ##########
integrated_linacc_x = integrate.cumtrapz(moving_imu[:, 8], moving_imu[:, 0], initial=0)
"""
# Calculate time differences for GPS data
delta_t = np.diff(moving_gps[:, 0])

delta_x = np.diff(moving_gps[:, 4])
delta_y = np.diff(moving_gps[:, 5])

# Calculate the velocity magnitude
gps_velocity = np.sqrt(delta_x**2 + delta_y**2) / delta_t

bias_x = np.mean(stationary_imu[:, 8])
adjusted_linacc_x = moving_imu[:, 8] - bias_x
adjusted_velocity = integrate.cumtrapz(adjusted_linacc_x, moving_imu[:, 0], initial=0)
threshold = 0.1  # Zero-velocity threshold, can be adjusted

for i in range(len(adjusted_velocity)):
    if adjusted_velocity[i] <= threshold:
        adjusted_velocity[i] = 0- adjusted_velocity[i] # Reset the velocity to zero
# Plotting
plt.close()
plt.plot(moving_imu[:, 0], integrated_linacc_x, 'b-', label='Integrated IMU Lin Acc X')
plt.plot(moving_gps[1:, 0], gps_velocity, 'g-', label='GPS Velocity')
plt.grid()
plt.legend()
plt.title('Velocity vs. Time')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.show()
plt.close()
plt.plot(moving_imu[:, 0], adjusted_velocity, 'b-', label='Adjusted Integrated IMU Lin Acc X')
plt.plot(moving_gps[:-1, 0], gps_velocity, 'g-', label='GPS Velocity')
plt.grid()
plt.legend()
plt.title('Adjusted Velocity vs. Time')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.show()
 """

""" ######### Dead Reckoning ##########
corrected_moving_complement = 0.99*corrected_yaw_intm +0.01* corrected_yaw_int_finalm
Ve = np.cos(corrected_moving_complement)
Vn = np.sin(corrected_moving_complement)
x_data = np.array([moving_imu[1,0], moving_imu[-1,0]])
y_data = np.array([integrated_linacc_x[1], integrated_linacc_x[-1]])
m,b= np.polyfit(x_data,y_data,1)
dist_lobf = np.absolute(moving_imu[:,0]*m - integrated_linacc_x[:] + b)/(np.sqrt(m**2+1))
#dist_lobf[12632:29632] = 0.5*dist_lobf[12632:29632]

Ve_mag = Ve * dist_lobf
Vn_mag = Vn * dist_lobf
pos_Ve = integrate.cumtrapz(Ve_mag, moving_imu[:, 0], initial=0)
pos_Vn = integrate.cumtrapz(Vn_mag, moving_imu[:, 0], initial=0)
pos_Ve = pos_Ve + moving_gps[0, 4]
pos_Vn = pos_Vn + moving_gps[0, 5]

plt.plot(moving_gps[:652, 4], moving_gps[:652, 5], 'c-', label='GPS X and Y Data')
plt.plot(pos_Ve[:25999], pos_Vn[:25999], 'g-', label='IMU Integrated Data')
plt.grid()
plt.legend()
plt.title('Position Data Using Complementary Filter')
plt.ylabel('Y Position (m)')
plt.xlabel('X Position (m)')
plt.show()

########## 3.1 ##########

plt.close()
plt.plot(moving_imu[:, 0], moving_imu[:, 9], 'c-', label='Y double dot')
plt.plot(moving_imu[:, 0], w_xdot, 'g-', label='W * Xdot')
plt.grid()
plt.legend()
plt.title('Y double dot compared to W*Xdot')
plt.ylabel('Acceleration (m/s^2)')
plt.xlabel('Time (s)')
plt.show() """
w_xdot = moving_imu[:, 7]*integrated_linacc_x
########## Xc ##########
dt_moving = np.empty((0))
for i in range(moving_imu.shape[0]-1):
    time_diff = moving_imu[i+1, 0] - moving_imu[i, 0]
    dt_moving = np.append(dt_moving, time_diff)
derived_wz = moving_imu[:len(dt_moving), 7] / dt_moving
if np.any(derived_wz == 0):
    print("Warning: derived_wz contains zero values!")
if np.any(np.isnan(w_xdot)) or np.any(np.isnan(derived_wz)):
    print("Warning: NaN values detected in arrays!")
epsilon = 1e-10  # Small non-zero value
derived_wz[derived_wz == 0] = epsilon

Xc_tot = (-1 * (w_xdot[:len(derived_wz)] / derived_wz))
Xc_tot = Xc_tot[np.logical_not(np.isnan(Xc_tot))]
Xc = np.mean(Xc_tot)
print(Xc)