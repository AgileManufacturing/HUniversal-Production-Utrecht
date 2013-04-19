echo -e "\033[36m===== Setting ROS_PACKAGE_PATH =====\033[0m"
source .export-rospath
echo -e "\033[36m===== Building C++ =====\033[0m"
catkin_make
source devel/setup.sh
rospack list > /dev/null
echo ""
echo -e "\033[36m===== Building JAVA =====\033[0m"
ant -buildfile src/java/build.xml
source build/java/.export-classpath
