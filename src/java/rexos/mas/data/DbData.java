/**
 * @file rexos/mas/data/DbData.java
 * @brief Data object for storing connection details for a database.
 * @date Created: 11 apr. 2013
 *
 * @author Peter Bonnema
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
package rexos.mas.data;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * Data object for storing connection details for a database.
 */
public class DbData implements MongoSaveable {
	/**
	 * @var String ip
	 * IP on which the database server is listening.
	 **/
	private String ip;

	/**
	 * @var int port
	 * Port at which the database server is listening.
	 **/
	private int port;

	/**
	 * @var String name
	 * Name of database.
	 **/
	private String name;

	/**
	 * Constructs a DbData object with the specified parameters.
	 * @param ip The ip of the database server.
	 * @param port The port of the database server.
	 * @param name The name of the database.
	 *
	 */
	public DbData(String ip, int port, String name) {
		this.ip = ip;
		this.port = port;
		this.name = name;
	}

	/**
	 * Constructs a DbData object from the data contained in the specified BasicDBObject.
	 * @param object The object
	 *
	 */
	public DbData(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * @see newDataClasses.DBSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start().add("ip", ip)
				.add("port", port).add("name", name).get();
	}

	/**
	 * @see newDataClasses.DBSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		ip = object.getString("ip");
		port = object.getInt("port");
		name = object.getString("name");
	}

	/**
	 * Returns the IP of this DbData object.
	 * @return the IP of this DbData object.
	 */
	public String getIp() {
		return ip;
	}

	/**
	 * Sets the IP for this DbData object.
	 * @param ip The ip to set
	 */
	public void setIp(String ip) {
		this.ip = ip;
	}

	/**
	 * Returns the port of this DbData object.
	 * @return the port of this DbData object.
	 */
	public int getPort() {
		return port;
	}

	/**
	 * Sets the port for this DbData object.
	 * @param port The port to set
	 */
	public void setPort(int port) {
		this.port = port;
	}

	/**
	 * Returns the name of this DbData object.
	 * @return the name of this DbData object.
	 */
	public String getName() {
		return name;
	}

	/**
	 * Sets the name for this DbData object.
	 * @param name The port to set
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Returns a String representing the data contained in this DbObject.
	 * @return A String representing the data contained in this DbObject.
	 */
	@Override
	public String toString() {
		return String
				.format("DbData [ip=%s, port=%s, name=%s]", ip, port, name);
	}
}
