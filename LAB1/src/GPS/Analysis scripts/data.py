import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from bagpy import bagreader

# Read the bag file
# bag = bagreader('/home/haoxinliu/catkin_ws/src/GPS/src/data/walking_data.bag')
bag = bagreader('/home/haoxinliu/catkin_ws/src/GPS/src/data/stationary_data.bag')
# Get the data for the /gps_data topic
data_path = bag.message_by_topic('/gps_data')
if data_path:
    gps_data = pd.read_csv(data_path)
    mean_easting = gps_data['UTM_easting'].mean()
    mean_northing = gps_data['UTM_northing'].mean()
    # Subtracting minimum values to start from (0,0),making the numbers smaller and potentially more interpretable in a plot.
    gps_data['error'] = np.sqrt((gps_data['UTM_easting'] - mean_easting)**2 + (gps_data['UTM_northing'] - mean_northing)**2)
    gps_data['UTM_easting'] = gps_data['UTM_easting'] - gps_data['UTM_easting'].min()
    gps_data['UTM_northing'] = gps_data['UTM_northing'] - gps_data['UTM_northing'].min()
    
    fig_error, ax_error = plt.subplots(figsize=(10, 8))
    ax_error.hist(gps_data['error'], bins=50, color='purple', edgecolor='black', alpha=0.7)
    ax_error.set_title('Histogram of Stationary GPS Error')
    ax_error.set_xlabel('Error Value (m)')
    ax_error.set_ylabel('Frequency')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()
    # Plotting the data
    # fig, ax = plt.subplots(figsize=(10, 8))
    # ax.scatter(gps_data['UTM_easting'], gps_data['UTM_northing'], s=50, label='GPS Points', color='blue')
    # ax.plot(gps_data['UTM_easting'].values, gps_data['UTM_northing'].values, color='red', label='Walking Path')
    # ax.plot(gps_data['UTM_easting'].values, gps_data['UTM_northing'].values, color='red', label='Stationary')
    # ax.set_title('Walking Path Visualized')
    # ax.set_title('Stationary Visualized')
    # ax.set_xlabel('UTM Easting (m)')
    # ax.set_ylabel('UTM Northing (m)')
    # ax.legend()
    # plt.show()
else:
    print("Couldn't retrieve the data.")
