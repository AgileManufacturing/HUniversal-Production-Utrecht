#!/usr/bin/env sh
# Assumes this file is run from the HUnivarsal-Production-Utrecht root
# and that the gateway server jar is also present in this folder.

if [ "$(id -u)" != "0" ]; then
   echo "\033[91mThis script must be run as root\033[0m" 1>&2
   exit 1
fi

tomcat_run() {
	for match in $(sudo find / -regextype sed -regex ".*/bin/startup\.sh")
	do
		if grep -q "Apache" $match
		then tcpath=$match
		fi
	done
	echo "\033[93mTomcat found at: $tcpath\033[0m"
	sudo sh $tcpath
}

run_build_script() {
	echo "\033[36m===== Building JAVA =====\033[0m"
	ant -buildfile src/REXOS/GatewayServer/build.xml
}

msg=$(sudo netstat -lnp | grep 8080)
if [ "$msg" = "" ]; then
	echo "\033[91m===== Tomcat is not running - Starting =====\033[0m"
	tomcat_run
fi

# Check wether the class files exist for the gateway server
# In case they do not, invoke ant to create them.

if [ ! -e ./build/REXOS/Gatewayserver/src/Main.class ]; then
	echo "Class files do not exist in ./build/REXOS/Gatewayserver/src, running ant now"
	run_build_script
fi

java -cp ./build/REXOS/Gatewayserver/src/gatewayserver gatewayserver.Main
