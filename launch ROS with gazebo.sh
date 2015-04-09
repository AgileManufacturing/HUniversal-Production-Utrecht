gnome-terminal 	--tab -e "bash -c \"roscore\"" \
		--tab -e "bash -c \"rosrun gazebo_ros gzserver --verbose\"" \
		--tab -e "bash -c \"rosrun gazebo_ros gzclient --verbose\"" \
		--tab -e "bash -c \"rosrun model_spawner_node model_spawner_node --spawnEquipletModel EQ2\"" \
		--tab -e "bash -c \"rosrun equiplet_node equiplet_node --isSimulated EQ2\"" \
		--tab -e "bash -c \"rosrun node_spawner_node node_spawner_node --isSimulated EQ2\"" \

