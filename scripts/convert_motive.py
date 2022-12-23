#!/usr/bin/env python

print("Starting conversion of Motive csv file to rosbag")

# Import libraries
import os
import pandas as pd
import datetime
import time
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped, Pose

# ROS parameters
# Format: default_param = rospy.get_param('default_param', 'default_value')
INPUT_FILE_NAME = rospy.get_param('~input_file_name', 'example_motive.csv')
OUTPUT_FILE_NAME = rospy.get_param('~output_file_name', 'example_motive.bag')
INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '../data/motive/')
OUTPUT_FILE_DIR = rospy.get_param('~output_file_dir', '../bags/motive/')
USE_STAMPED_MESSAGE = rospy.get_param('~use_stamped_message', True)
BODY_NAMES_PREFIX = rospy.get_param('~body_names_prefix', 'rigid_body_')
LIST_OF_BODY_NAMES = rospy.get_param('~list_of_body_names', [])
INPUT_FILE_PATH = rospy.get_param('~input_file_path', INPUT_FILE_DIR + INPUT_FILE_NAME)
OUTPUT_FILE_PATH = rospy.get_param('~output_file_path', OUTPUT_FILE_DIR + OUTPUT_FILE_NAME)

# Read the csv file
df = pd.read_csv(INPUT_FILE_PATH)
print("CSV file opened: " + os.path.abspath(INPUT_FILE_PATH))

# Separate head and content
df_head = df.head()
df_no_head = pd.read_csv(INPUT_FILE_PATH, skiprows=df_head.shape[0]+1)

# Get the time and convert to epoch
index_capture_start_time = df_head.columns.get_loc('Capture Start Time')
start_time = df_head.columns.to_list()[index_capture_start_time + 1]
time_datetime = datetime.datetime.strptime(start_time, '%Y-%m-%d %H.%M.%S.%f %p')
# epoch = time_datetime.timestamp()             # Python 3 only
epoch = time.mktime(time_datetime.timetuple())  # Works on Python 2 and 3

# Remove useless columns
df_no_head = df_no_head.dropna(axis=1, how='all')
df_no_head = df_no_head.iloc[:, 1:]

# Replace time column with epoch
df_no_head['Time (Seconds)'] += [epoch for i in range(df_no_head.shape[0])]

# Rename columns
nb_rigid_bodies = int((df_no_head.shape[1] - 1) / 7)
list_of_columns_name = ['Epoch']
for i in range(1, nb_rigid_bodies+1):
    list_of_columns_name.append('x'+str(i))
    list_of_columns_name.append('y'+str(i))
    list_of_columns_name.append('z'+str(i))
    list_of_columns_name.append('qx'+str(i))
    list_of_columns_name.append('qy'+str(i))
    list_of_columns_name.append('qz'+str(i))
    list_of_columns_name.append('qw'+str(i))
df_no_head.columns = list_of_columns_name

# If the bag file does not exist, create it
file_exists = os.path.exists(OUTPUT_FILE_PATH)
if not file_exists:
    with open(OUTPUT_FILE_PATH, "w") as f: 
        f.write("") 

# Convert to rosbag
with rosbag.Bag(OUTPUT_FILE_PATH, 'w') as bag:

    # Iterate on each row
    for row in range(df_no_head.shape[0]):

        # Set timestamp
        timestamp = rospy.Time.from_sec(df_no_head['Epoch'][row])

        # Create one pose message for each rigid body
        for i in range(1, nb_rigid_bodies+1):
            if USE_STAMPED_MESSAGE:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = timestamp
                pose_msg.pose.position.x = df_no_head['x'+str(i)][row]
                pose_msg.pose.position.y = df_no_head['y'+str(i)][row]
                pose_msg.pose.position.z = df_no_head['z'+str(i)][row]
                pose_msg.pose.orientation.x = df_no_head['qx'+str(i)][row]
                pose_msg.pose.orientation.y = df_no_head['qy'+str(i)][row]
                pose_msg.pose.orientation.z = df_no_head['qz'+str(i)][row]
                pose_msg.pose.orientation.w = df_no_head['qw'+str(i)][row]
                bag.write(BODY_NAMES_PREFIX + str(i), pose_msg, timestamp)
            else:
                pose_msg = Pose()
                pose_msg.position.x = df_no_head['x'+str(i)][row]
                pose_msg.position.y = df_no_head['y'+str(i)][row]
                pose_msg.position.z = df_no_head['z'+str(i)][row]
                pose_msg.orientation.x = df_no_head['qx'+str(i)][row]
                pose_msg.orientation.y = df_no_head['qy'+str(i)][row]
                pose_msg.orientation.z = df_no_head['qz'+str(i)][row]
                pose_msg.orientation.w = df_no_head['qw'+str(i)][row]
                bag.write(BODY_NAMES_PREFIX + str(i), pose_msg, timestamp)
    
    print("Bag file created: " + os.path.abspath(OUTPUT_FILE_PATH))

print("Conversion done")