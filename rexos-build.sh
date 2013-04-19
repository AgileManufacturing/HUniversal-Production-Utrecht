#!/usr/bin/env sh
REXOS_BUILD_TARGET=""
function usage() {
	echo "Builds or cleans the cpp and java parts for the REXOS project."
	echo "Usage: source rexos-build.sh [-c]"
	echo "Defaults to build, use -c to clean."
}

#Have to clear OPTIND because this file as sourced and OPTIND is only cleared when creating a new shell.
OPTIND=0
while getopts ":ch" opt; do
	case $opt in
		c)
			REXOS_BUILD_TARGET="clean"
			;;
		h)
			usage
			return
			;;
		\?)
			usage
			return
			;;
	esac
done

echo -e "\033[36m===== Setting ROS_PACKAGE_PATH =====\033[0m"
. ./.export-rospath

echo -e "\033[36m===== Building C++ =====\033[0m"
catkin_make $REXOS_BUILD_TARGET
if [ "$REXOS_BUILD_TARGET" != "clean" ];
then
	. ./devel/setup.sh
fi

#rosrun apparently caches its module list. Force an update so tab complete works.
rospack list > /dev/null

echo ""
echo -e "\033[36m===== Building JAVA =====\033[0m"
ant -buildfile src/java/build.xml $REXOS_BUILD_TARGET

if [ "$REXOS_BUILD_TARGET" != "clean" ];
then
	. ./build/java/.export-classpath
fi

#Have to clear OPTIND because this file as sourced and OPTIND is only cleared when creating a new shell.
OPTIND=0
unset REXOS_BUILD_TARGET