#!/usr/bin/env sh
# Assumes this file is run from the HUnivarsal-Production-Utrecht root
# and that the gateway server jar is also present in this folder.

if [ "$(id -u)" != "0" ]; then
   echo "\033[91mThis script must be run as root\033[0m" 1>&2
   exit 1
fi

tomcat_run() {
	tcdir=$(sudo find / -type d -name "tomcat")
	echo "\033[93mTomcat found at: $tcdir\033[0m"
	sudo sh $tcdir/bin/startup.sh
}

msg=$(sudo netstat -lnp | grep 8080)
if [ "$msg" = "" ];
then
	echo "\033[91m===== Tomcat is not running - Starting =====\033[0m"
	tomcat_run
fi

java -classpath ./src/GatewayServer/bin Main
