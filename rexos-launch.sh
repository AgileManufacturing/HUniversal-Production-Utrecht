#!/usr/bin/env sh
REXOS_MAS_TARGET=""
REXOS_ROS_TARGET=""
REXOS_GRID_HOST=""
REXOS_EQ_NR=""
REXOS_LOCALIP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')

function usage() {
	echo "Launches the REXOS platform"
	echo "Usage: source rexos-launch.sh [-g] [-e ipaddress:port] [-m modulename]"
	echo "Defaults to launching the grid agents, use [-e ipaddress:port] to launch an equiplet"
	echo "Launches essential ros nodes by default only, use -m to specify a module launch file (ex. gripper -or- pen)"
}



function run() {
	# Start essential nodes (equiplet node)
	# roslaunch -- launchfile
	#roslaunch essentials.launch

	# *Determine installed modules (database)* // Not going to happen now
	# roslaunch -- launchfile
	if [ "$REXOS_ROS_TARGET" != "" ];
	then
		roslaunch $REXOS_ROS_TARGET
	fi

	# Start MAS (using ant)
	echo ""
	echo -e "\033[36m===== Launching MAS =====\033[0m"
	ant -buildfile src/REXOS/MAS/build.xml $REXOS_MAS_TARGET -Dgridhost=$REXOS_GRID_HOST -Dequipletnumber=$REXOS_EQ_NR -Dlocalip=$REXOS_LOCALIP
}

OPTIND=0
while getopts "n:m:e:gt" opt; do
	case "$opt" in
		n)	REXOS_EQ_NR="$OPTARG";;
		g)	REXOS_MAS_TARGET="Launch-Grid"; run;;
		e)	REXOS_MAS_TARGET="Launch-EQ"; REXOS_GRID_HOST="$OPTARG"; run;;
		m)	REXOS_ROS_TARGET="$OPTARG.launch";;
		\?)	usage;;
	esac
done

OPTIND=0
unset REXOS_MAS_TARGET
unset REXOS_ROS_TARGET
unset REXOS_GRID_HOST
unset REXOS_EQ_NR
unset REXOS_LOCALIP
