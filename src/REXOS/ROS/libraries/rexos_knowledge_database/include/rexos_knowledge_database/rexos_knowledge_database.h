/**
 * @file DatabaseConnection.h
 * @brief Coordinate system for communication between nodes
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#ifndef REXOS_KNOWLEDGE_DATABASE_H
#define REXOS_KNOWLEDGE_DATABASE_H

#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>

#include <memory>

// TEMP!
#define MYSQL_SERVER "tcp://127.0.0.1:3306"
#define MYSQL_DATABASE "equiplet2"
#define MYSQL_USERNAME "rexos"
#define MYSQL_PASSWORD "soxer"

/**
 * @brief this class provides a system to translate coordinates from module coordinates to equiplet coordinates.
 */
namespace rexos_knowledge_database {
	std::unique_ptr<sql::Connection> connect();
	
}
#endif /* REXOS_KNOWLEDGE_DATABASE_H */
