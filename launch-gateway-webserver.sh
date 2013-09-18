#!/usr/bin/env sh
# Assumes this file is run from the HUnivarsal-Production-Utrecht root
# and that the gateway server jar is also present in this folder.

tomcat_run() {
	# find tomcat (TODO)
	sudo sh ~/Programs/tomcat/bin/startup.sh
}

msg=$(sudo netstat -lnp | grep 8080)
if [ "$msg" = "" ];
then
	echo "Tomcat is not running - Starting"
	tomcat_run
fi

java -jar ./gatewayserver.jar
