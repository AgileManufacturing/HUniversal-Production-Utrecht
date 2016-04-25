#!/usr/bin/env sh
###################### SETTINGS ######################
keepWindowOpenAfterCommandEnds=false
defaultManufacturer="HU"
defaultTypeNumber="workplane_type_A"
defaultSerialNumber="1"
###################### END OF SETTINGS ######################
###################### VARS ######################
launchedPrimaryInfrastructureWithSimulation=false
cancelAction=false
###################### END OF VARS ######################
###################### FUNCTIONS ######################
function getEquipletName() {
	echo -n "Choose equipletName: "
	read equipletName
}
function getPartName() {
	echo -n "Choose partName: "
	read partName
}
function getModuleIdentifier() {
	echo -n "Choose manufacturer (default: $defaultManufacturer): "
	read manufacturer
	if [ "$manufacturer" == "" ]; then
		manufacturer="$defaultManufacturer"
	fi
	echo -n "Choose typeNumber (default: $defaultTypeNumber): "
	read typeNumber
	if [ "$typeNumber" == "" ]; then
		typeNumber="$defaultTypeNumber"
	fi
	echo -n "Choose serialNumber (default: $defaultSerialNumber): "
	read serialNumber
	if [ "$serialNumber" == "" ]; then
		serialNumber="$defaultSerialNumber"
	fi
	moduleIdentifier="$manufacturer|$typeNumber|$serialNumber"
}
function getPosition() {
	echo -n "Choose positionX: "
	read positionX
	echo -n "Choose positionY: "
	read positionY
}
function getOriginPlacementType() {
	echo "Choose origin placement type: "
	echo "1: Relative to equiplet origin (relativeToEquipletOrigin)"
	echo "2: Relative to module origin (relativeToModuleOrigin)"
	echo "3: Relative to part origin (relativeToIdentifier)"
	echo "4: Relative to world origin (relativeToWorldOrigin)"
	echo "Q: Cancel"
	echo ""
	echo -n "Choose option: "
	read chosenOption
	if [ "$chosenOption" == 1 ]; then
		originPlacementType=RELATIVE_TO_EQUIPLET_ORIGIN
		getEquipletName
		relativeTo="$equipletName"
	elif [ "$chosenOption" == 2 ]; then
		originPlacementType=RELATIVE_TO_MODULE_ORIGIN
		getModuleIdentifier
		relativeTo="$moduleIdentifier"
	elif [ "$chosenOption" == 3 ]; then
		originPlacementType=RELATIVE_TO_PART_ORIGIN
		getPartName
		relativeTo="$partName"
	elif [ "$chosenOption" == 4 ]; then
		originPlacementType=RELATIVE_TO_WORLD_ORIGIN
		relativeTo=""
	else
		echo "Cancelling"
		cancelAction=true
	fi
}
function getPose() {
	echo -n "Choose positionX: "
	read positionX
	echo -n "Choose positionY: "
	read positionY
	echo -n "Choose positionZ: "
	read positionZ
	echo -n "Choose rotationX: "
	read rotationX
	echo -n "Choose rotationY: "
	read rotationY
	echo -n "Choose rotationZ: "
	read rotationZ
}

function launchPrimaryInfrastructureWithSimulation() {
	echo "Launching primary infrastructure with simulation"
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" 	-e "bash -c \"roscore\";bash" \
			--tab -t "gzserver" -e "bash -c \"rosrun gazebo_ros gzserver --verbose world.sdf\";bash" \
			--tab -t "gzclient" -e "bash -c \"rosrun gazebo_ros gzclient --verbose\";bash"
	else
		gnome-terminal --title="Primary infrastructure" \
			--tab -t "roscore" 	-e "bash -c \"roscore\"" \
			--tab -t "gzserver" -e "bash -c \"rosrun gazebo_ros gzserver --verbose world.sdf\"" \
			--tab -t "gzclient" -e "bash -c \"rosrun gazebo_ros gzclient --verbose\""
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
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --isShadow --spawnEquipletModel -x $positionX -y $positionY $equipletName\";bash"
	else
		gnome-terminal --title="$equipletName" \
			--tab -t "equiplet_node" 		-e "bash -c \"rosrun equiplet_node equiplet_node --isShadow $equipletName\"" \
			--tab -t "node_spawner_node" 	-e "bash -c \"rosrun node_spawner_node node_spawner_node --isShadow $equipletName\"" \
			--tab -t "environment_cache" 	-e "bash -c \"rosrun environment_cache environment_cache --isShadow $equipletName\"" \
			--tab -t "model_spawner_node" 	-e "bash -c \"rosrun model_spawner_node model_spawner_node --isShadow --spawnEquipletModel -x $positionX -y $positionY $equipletName\""
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

function launchPartModel() {
	echo "Launching part model"
	getPartName
	partNameToSpawn="$partName"
	if [ "$cancelAction" == true ] ; then
		return
	fi
	getOriginPlacementType
	if [ "$cancelAction" == true ] ; then
		return
	fi
	getPose
	
	echo "bash -c \"rosrun model_spawner_node model_spawner_node --spawnPartModel --partName $partNameToSpawn \
				--originPlacementType $originPlacementType \\\"$relativeTo\\\" \
				-x $positionX -y $positionY -z $positionZ --pitch $rotationX --roll $rotationY --yaw $rotationZ\""
	
	if [ "$keepWindowOpenAfterCommandEnds" == true ]; then
		gnome-terminal --title="$partName" \
			--tab -t "$partName" -e "bash -c \"rosrun model_spawner_node model_spawner_node --isShadow --spawnPartModel --partName $partNameToSpawn \
				--originPlacementType $originPlacementType \\\"$relativeTo\\\" \
				-x $positionX -y $positionY -z $positionZ --pitch $rotationX --roll $rotationY --yaw $rotationZ\";bash"
	else
		gnome-terminal --title="$partName" \
			--tab -t "$partName" -e "bash -c \"rosrun model_spawner_node model_spawner_node --isShadow --spawnPartModel --partName $partNameToSpawn \
				--originPlacementType $originPlacementType \\\"$relativeTo\\\" \
				-x $positionX -y $positionY -z $positionZ --pitch $rotationX --roll $rotationY --yaw $rotationZ\""
	fi
	
	
}
###################### END OF FUNCTIONS ######################


###################### USER INTERFACE ######################
echo "This script allows you to dynamically launch ROS components of REXOS";

echo "Step 1: choose primary infrastructure:"
echo "1: Launch primary infrastructure with simulation"
echo "2: Launch primary infrastructure without simulation"
echo "3: Primary infrastructure with simulation is already running"
echo "4: Primary infrastructure without simulation is already running"
echo "Q: Exit"
echo ""
echo -n "Choose option: "
read chosenOption

if [ "$chosenOption" == 1 ]; then
	launchPrimaryInfrastructureWithSimulation
elif [ "$chosenOption" == 2 ]; then
	launchPrimaryInfrastructureWithoutSimulation
elif [ "$chosenOption" == 3 ]; then
	launchedPrimaryInfrastructureWithSimulation=true
elif [ "$chosenOption" == 4 ]; then
	launchedPrimaryInfrastructureWithSimulation=false
elif [ "$chosenOption" == "Q" ] || [ "$chosenOption" == "q" ]; 
then
	echo "Exiting"
	return
else 
	echo "Invalid option"
	return
fi

exitRequested=false 

while [ "$exitRequested" == false ];
do
	cancelAction=false
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
	elif [ "$chosenOption" == 8 ] && [ "$launchedPrimaryInfrastructureWithSimulation" == true ]; then
		launchPartModel
	elif [ "$chosenOption" == "Q" ] || [ "$chosenOption" == "q" ]; then
		echo "Exiting"
		exitRequested=true
	else echo "Invalid option"
	fi
done
###################### END OF USER INTERFACE ######################
