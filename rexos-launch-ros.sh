#!/usr/bin/env sh
###################### VARS ######################
launchedPrimaryInfrastructureWithSimulation=false
keepWindowOpenAfterCommandEnds=true

###################### END OF VARS ######################
###################### FUNCTIONS ######################
function getEquipletName() {
	echo -n "Choose equipletName: "
	read equipletName
}
function getPosition() {
	echo -n "Choose positionX: "
	read positionX
	echo -n "Choose positionY: "
	read positionY
}

function launchPrimaryInfrastructureWithSimulation() {
	echo "Launching primary infrastructure with simulation"
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" 	-e "bash -c \"roscore\";bash" \
			--tab -t "gzclient" 	-e "bash -c \"rosrun gazebo_ros gzserver --verbose\";bash" \
			--tab -t "roscore" 	-e "bash -c \"rosrun gazebo_ros gzclient --verbose\";bash"
	else
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" 	-e "bash -c \"roscore\"" \
			--tab -t "gzserver" 	-e "bash -c \"rosrun gazebo_ros gzserver --verbose\"" \
			--tab -t "gzclient" 	-e "bash -c \"rosrun gazebo_ros gzclient --verbose\""
	fi
	launchedPrimaryInfrastructureWithSimulation=true
}

function launchPrimaryInfrastructureWithoutSimulation() {
	echo "Launching primary infrastructure without simulation"
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" -e "bash -c \"roscore\";bash"
	else
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" -e "bash -c \"roscore\""
	fi
}

function launchEquipletWithPrimaryNodes() {
	echo "Launching equiplet with primary nodes"
	getEquipletName
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 	-e "bash -c \"rosrun equiplet_node equiplet_node $equipletName\";bash" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node $equipletName\";bash" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node $equipletName\"" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node $equipletName\"" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache $equipletName\""
	fi
}

function launchEquipletWithoutPrimaryNodes() {
	echo "Launching equiplet without primary nodes"
	getEquipletName
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -e "bash -c \"rosrun equiplet_node equiplet_node $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -e "bash -c \"rosrun equiplet_node equiplet_node $equipletName\""
	fi
}

function launchSimulatedEquipletWithPrimaryNodes() {
	echo "Launching simulated equiplet with primary nodes"
	getEquipletName
	getPosition
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node --isSimulated $equipletName\";bash" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node --isSimulated $equipletName\";bash" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache --isSimulated $equipletName\";bash" \
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --spawnEquipletModel -x $positionX -y $positionY $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node --isSimulated $equipletName\"" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node --isSimulated $equipletName\"" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache --isSimulated $equipletName\"" \
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --spawnEquipletModel -x $positionX -y $positionY $equipletName\""
	fi
}

function launchSimulatedEquipletWithoutPrimaryNodes() {
	echo "Launching simulated equiplet without primary nodes"
	getEquipletName
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" -e "bash -c \"rosrun equiplet_node equiplet_node --isSimulated $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" -e "bash -c \"rosrun equiplet_node equiplet_node --isSimulated $equipletName\""
	fi
}

function launchShadowEquipletWithPrimaryNodes() {
	echo "Launching shadow equiplet with primary nodes"
	getEquipletName
	getPosition
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node --isShadow $equipletName\";bash" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node --isShadow $equipletName\";bash" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache --isShadow $equipletName\";bash" \
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --spawnEquipletModel -x $positionX -y $positionY $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node --isShadow $equipletName\"" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node --isShadow $equipletName\"" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache --isShadow $equipletName\"" \
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --spawnEquipletModel -x $positionX -y $positionY $equipletName\""
	fi
}

function launchShadowEquipletWithoutPrimaryNodes() {
	echo "Launching shadow equiplet without primary nodes"
	getEquipletName
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" -e "bash -c \"rosrun equiplet_node equiplet_node --isShadow $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" -e "bash -c \"rosrun equiplet_node equiplet_node --isShadow $equipletName\""
	fi
}
###################### END OF FUNCTIONS ######################


###################### USER INTERFACE ######################
echo "This script allows you to dynamically launch ROS components of REXOS";

echo "Step 1: choose primary infrastructure:"
echo "1: Launch primary infrastructure with simulation"
echo "2: Launch primary infrastructure without simulation"
echo "Q: Exit"
echo ""
echo -n "Choose option: "
read chosenOption

if [ "$chosenOption" == 1 ]; then
	launchPrimaryInfrastructureWithSimulation
elif [ "$chosenOption" == 2 ]; then
	launchPrimaryInfrastructureWithoutSimulation
elif [ "$chosenOption" == "Q" ] || [ "$chosenOption" == "q" ]; 
then
	echo "Exiting"
	exit
else 
	echo "Invalid option"
	exit
fi

exitRequested=false 

while [ "$exitRequested" == false ];
do
	echo ""
	echo ""
	echo "Step 2: choose action:"
	echo "1: Launch equiplet with primary nodes"
	echo "2: Launch equiplet without primary nodes"
if [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
	echo "3: Launch simulated equiplet with primary nodes"
	echo "4: Launch simulated equiplet without primary nodes"
	echo "5: Launch shadow equiplet with primary nodes"
	echo "6: Launch shadow equiplet without primary nodes"
	echo "7: Launch equiplet model and module models only"
	echo "8: Launch part model"
fi
	echo "Q: Exit"
	echo ""
	echo -n "Choose option: "
	read chosenOption

	if [ "$chosenOption" == 1 ]; then
		launchEquipletWithPrimaryNodes
	elif [ "$chosenOption" == 2 ]; then
		launchEquipletWithoutPrimaryNodes
	elif [ "$chosenOption" == 3 ] && [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
		launchSimulatedEquipletWithPrimaryNodes
	elif [ "$chosenOption" == 4 ] && [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
		launchSimulatedEquipletWithoutPrimaryNodes
	elif [ "$chosenOption" == 5 ] && [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
		launchShadowEquipletWithPrimaryNodes
	elif [ "$chosenOption" == 6 ] && [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
		launchShadowEquipletWithoutPrimaryNodes
	elif [ "$chosenOption" == "Q" ] || [ "$chosenOption" == "q" ]; then
		echo "Exiting"
		exitRequested=true
	else echo "Invalid option"
	fi
done
###################### END OF USER INTERFACE ######################
