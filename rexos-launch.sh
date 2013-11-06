#!/usr/bin/env sh
REXOS_MAS_TARGET=""
REXOS_ROS_TARGET=""
REXOS_GRID_HOST=""
REXOS_EQ_ID=""
REXOS_LOCALIP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')

function usage() {
	echo "Launches the REXOS platform"
	echo "Usage: source rexos-launch.sh [-i|--equipletid equiplet id] [-g|--grid] [-e|--equiplet ipaddress:port] [-l|--localip local ipaddress] [-m|--module modulename]"
	echo "Defaults to launching the grid agents, use [-i|--equipletid equiplet id] [-e|--equiplet ipaddress:port] [-l|--localip local ipaddress] to launch an equiplet"
	echo "Launches essential ros nodes by default only, use [-m|--module modulename] to specify a module launch file (ex. gripper -or- pen)"
}

function runros() {
	# Start essential nodes (equiplet node)
	# roslaunch -- launchfile
	gnome-terminal -e "bash -c 'roslaunch essentials.launch'"

	# Delay for 5 seconds to allow the essential nodes to start
	sleep 5

	# *Determine installed modules (database)* // Not going to happen now
	# roslaunch -- launchfile
	if [ "$REXOS_ROS_TARGET" != "" ];
	then
		gnome-terminal -e "bash -c 'roslaunch $REXOS_ROS_TARGET'"
	fi
}

function run() {
	# Start MAS (using ant)
	echo ""
	echo -e "\033[36m===== Launching MAS =====\033[0m"
	ant -buildfile src/REXOS/MAS/build.xml $REXOS_MAS_TARGET -Dgridhost=$REXOS_GRID_HOST -Dequipletnumber=$REXOS_EQ_ID -Dlocalip=$REXOS_LOCALIP
}

OPTIND=0
while [ "$1" != "" ]; do
	case "$1" in
		-i | --equipletid )	shift; REXOS_EQ_ID="$1";;
		-g | --grid )		REXOS_MAS_TARGET="Launch-Grid"; run;;
		-e | --equiplet )	REXOS_MAS_TARGET="Launch-EQ"; shift; REXOS_GRID_HOST="$1"; runros; run;;
		-m | --module )		REXOS_ROS_TARGET="$OPTARG.launch";;
		-l | --localip )	shift; REXOS_LOCALIP="$1"; echo "local ip: $REXOS_LOCALIP";;
		-h | --help | \? )	usage;;
	esac
	shift
done

OPTIND=0
unset REXOS_MAS_TARGET
unset REXOS_ROS_TARGET
unset REXOS_GRID_HOST
unset REXOS_EQ_ID
unset REXOS_LOCALIP
