# Starting from a clean slate

First, we will want to check if there are any updates. Sometimes this is already done for you assuming you connected to internet during the ubuntu system setup guide.

	sudo apt update

	sudo apt upgrade

# Install git

To contribute changes, you will need to have a git client installed. This program will be used to track and upload your changes.

	sudo apt install git

# Cloning the repository

You need to clone (download) a copy of the repository onto your computer so you can make changes.

First clone the upstream (MIL’s fork) version of the repo. It is recommended that you first create a catkin workspace and clone it into the src or that workspace.

	mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

Then clone the repo:

(the --recurse-submodules is essential, as that is what pulls the submodules, -j8 is a speed optimization that downloads up to 8 submodules at a time)

	git clone --recurse-submodules -j8 git@github.com:andrespulido8/bathydrone.git

# Gazebo 

Install Gazebo

	curl -sSL http://get.gazebosim.org | sh

Install ROS Bridge for Gazebo

	sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
You can check for any missing dependencies using rosdep:

	rosdep update
	rosdep check --from-paths . --ignore-src --rosdistro noetic

You can automatically install the missing dependencies using rosdep via debian install:

	rosdep install --from-paths . --ignore-src --rosdistro noetic -y

Build the packages
	cd ~/catkin_ws/
	catkin_make 

Test gazebo with ROS

	roscore &
	rosrun gazebo_ros gazebo
