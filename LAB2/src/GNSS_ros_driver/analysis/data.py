import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from bagpy import bagreader
from matplotlib.ticker import FormatStrFormatter

bag = bagreader('/home/haoxinliu/catkin_ws/src/eece5554/LAB2/src/GNSS_ros_driver/data/moving.bag')
# Get the data for the /rtk topic
data_path = bag.message_by_topic('/rtk')
if data_path:
    rtk = pd.read_csv(data_path)
    print(rtk.columns)
    window_size = 10  # You can adjust this value based on your requirements
    rtk['moving_mean_easting'] = rtk['utmEasting'].rolling(window=window_size).mean()
    rtk['moving_mean_northing'] = rtk['utmNorthing'].rolling(window=window_size).mean()

    # Calculate the error based on the moving mean
    rtk['error'] = np.sqrt((rtk['utmEasting'] - rtk['moving_mean_easting'])**2 + 
                           (rtk['utmNorthing'] - rtk['moving_mean_northing'])**2)
    
    # For plotting purposes
    rtk['utmEasting'] = rtk['utmEasting'] - rtk['utmEasting'].min()
    rtk['utmNorthing'] = rtk['utmNorthing'] - rtk['utmNorthing'].min()

    fig_error, ax_error = plt.subplots(figsize=(10, 8))
    ax_error.hist(rtk['error'].dropna(), bins=50, color='purple', edgecolor='black', alpha=0.7)  # dropna() to remove NaN values due to rolling
    ax_error.set_title('Histogram of Moving GPS Error')
    ax_error.set_xlabel('Error Value (m)')
    ax_error.set_ylabel('Frequency')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10,8))
    plt.scatter(rtk['utmEasting'], rtk['utmNorthing'], c='blue', s=5)  # s represents marker size
    plt.title('Easting vs Northing')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    """ mean_easting = rtk['utmEasting'].mean()
    mean_northing = rtk['utmNorthing'].mean()
    # Subtracting minimum values to start from (0,0), making the numbers smaller for plotting.
    rtk['error'] = np.sqrt((rtk['utmEasting'] - mean_easting)**2 + (rtk['utmNorthing'] - mean_northing)**2)
    rtk['utmEasting'] = rtk['utmEasting'] - rtk['utmEasting'].min()
    rtk['utmNorthing'] = rtk['utmNorthing'] - rtk['utmNorthing'].min()
    fig_error, ax_error = plt.subplots(figsize=(10, 8))
    ax_error.hist(rtk['error'], bins=50, color='purple', edgecolor='black', alpha=0.7)
    ax_error.set_title('Histogram of Stationary GPS Error')
    ax_error.set_xlabel('Error Value (m)')
    ax_error.set_ylabel('Frequency')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show() """
    # ... [The initial part of your code remains the same]

    # Longitude vs Latitude Plotprint("Longitude range:", rtk['longitude'].min(), "to", rtk['longitude'].max())
    print("Longitude range:", rtk['longitude'].min(), "to", rtk['longitude'].max())
    print("Latitude range:", rtk['latitude'].min(), "to", rtk['latitude'].max())
    fig_latlong, ax_latlong = plt.subplots(figsize=(10, 8))

    # Plot the GPS points
    ax_latlong.scatter(rtk['latitude'], rtk['longitude'], s=50, label='GPS Points', color='blue')
    ax_latlong.plot(rtk['latitude'].values, rtk['longitude'].values, color='red', label='Path')

    # Set the title and legend
    ax_latlong.set_title('Longitude vs Latitude Visualized')
    ax_latlong.legend()

    # Adjust the axes limits
    ax_latlong.set_xlim(42.33840991333334, 42.338648766666665)  # Latitude range
    ax_latlong.set_ylim(-71.08867894833334, -71.088433495)  # Longitude range

    # Set the tick intervals
    ax_latlong.xaxis.set_ticks(np.arange(42.33840991333334, 42.338648766666665, 0.0001))
    ax_latlong.yaxis.set_ticks(np.arange(-71.08867894833334, -71.088433495, 0.0002))

    # Show the grid, layout adjustments, and display the plot
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()



    # Time vs Altitude Plot
    # First, we need to convert the time column to a more readable format (if not already in one)
    # rtk['Time'] = pd.to_datetime(rtk['Time'])  # Uncomment this line if 'Time' is not in datetime format

    fig_timealt, ax_timealt = plt.subplots(figsize=(10, 8))
    ax_timealt.plot(rtk['Time'].values, rtk['altitude'].values, color='green', label='Altitude over Time')
    ax_timealt.set_title('Altitude over Time')
    ax_timealt.set_xlabel('Time')
    ax_timealt.set_ylabel('Altitude (m)')
    ax_timealt.legend()
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.tight_layout()
    plt.show()


else:
    print("Couldn't retrieve the data.")
    
