#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include "ros/ros.h"

#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <rexos_configuration/Configuration.h>

namespace rexos_knowledge_database{
	std::unique_ptr<sql::Connection> connect(){
		sql::Driver* driver = get_driver_instance();
		std::unique_ptr<sql::Connection> apConnection(
			driver->connect(
					(
						std::string("tcp://") + rexos_configuration::Configuration::getProperty("knowledgeDatabase/ip").asString() + 
						std::string(":") + boost::lexical_cast<std::string>(rexos_configuration::Configuration::getProperty("knowledgeDatabase/port").asInt())
					),
					rexos_configuration::Configuration::getProperty("knowledgeDatabase/username").asString(), 
					rexos_configuration::Configuration::getProperty("knowledgeDatabase/password").asString())
		);
		apConnection->setSchema(MYSQL_DATABASE);
		return apConnection;
	}
}
