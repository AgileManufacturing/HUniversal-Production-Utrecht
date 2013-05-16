if [ ${BASH_ARGV[0]} = "setup.sh" ]
then
	echo "sourcing fuerte setup.sh and adding $PWD to ROS_PACKAGE_PATH."
	source /opt/ros/fuerte/setup.sh
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PWD}
	echo "ROS_PACKAGE_PATH is now: $ROS_PACKAGE_PATH"
else
	echo "Execute from repo root please!"
fi
