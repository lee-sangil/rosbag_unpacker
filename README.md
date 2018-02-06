# Rosbag Unpacker

ROS package to extract a rosbag into a collection of images. Images are ordered in a time sequence. The timestamp for each image is assigned according to the filename of image. 

The bag will publish the images to topic as specified.

Tested in ROS Kinetic.

## Installation

In your ROS_PACKAGE_PATH (check your environment variable ROS_PACKAGE_PATH):

	git clone https://github.com/lee-sangil/rosbag_unpacker.git rosbag_unpacker
	cd ${catkin_workspace}
	catkin_make

## Usage:

	roslaunch rosbag_unpacker rosbag_unpacker.launch path:=/PATH/TO/DIRECTORY/ file:=INPUT.BAG

- `PATH_TO_FOLDER`: Path to the folder with the images and/or the depths
- `PATH_TO_INPUT_BAG`: Path to the bag (including the filename e.g. directory/filename.bag)

