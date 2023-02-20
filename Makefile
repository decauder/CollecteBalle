.PHONY: prerun run test

.ONESHELL:

# WARNING: Before running the build, make sure, that ROS_DOMAIN_ID=0..101 is exported
# in your shell. Otherwise, the nodes will not be able to communicate. OR you can set `export ROS_LOCALHOST_ONLY=1`

prerun:
	$(shell . ./install/setup.sh)
	$(info Running with ROS_DOMAIN_ID = ${ROS_DOMAIN_ID})

build_all:
	colcon build
	$(shell . ./install/setup.sh)

run_court: prerun
	ros2 launch tennis_court tennis_court.launch.py

run_camera: prerun
	ros2 run top_camera zenith_camera_subscriber

test:
	cd ws; colcon test --packages-select top_camera robot_control --event-handlers console_cohesion+
	# cd ws; colcon test --event-handlers console_cohesion+
	