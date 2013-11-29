#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include "ros/ros.h"

#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	std::unique_ptr<sql::Connection> connect(){
		sql::Driver* driver = get_driver_instance();
		std::unique_ptr<sql::Connection> apConnection(
			driver->connect(MYSQL_SERVER, MYSQL_USERNAME, MYSQL_PASSWORD)
		);
		apConnection->setSchema(MYSQL_DATABASE);
		return apConnection;
	}
}
