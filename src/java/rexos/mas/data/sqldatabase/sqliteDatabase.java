/**
 * @file sqliteDatabase.java
 * @brief Class in which an SQLite database is created and items can be inserted
 *        into it.
 * @date Created: 02-04-2013
 * 
 * @author Theodoor de Graaff
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.data.sqldatabase;

import rexos.libraries.log.Logger;
import rexos.mas.data.LogMessage;

import java.sql.Connection;
import java.sql.Driver;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.List;

public class sqliteDatabase{
	static final String JDBC_DRIVER = "org.sqlite.JDBC";
	static final String DB_URL = "jdbc:sqlite:";
	String DB_FILE = "default.sqlite";
	// Database credentials
	static final String USER = "username";
	static final String PASS = "password";
	Connection conn = null;
	Statement stmt = null;

	public sqliteDatabase(String filename){
		if (filename != null){
			filename = filename.replaceAll("[\\/:*?\"<>|]","");
			
			DB_FILE = filename;
		}
		try{
			Driver d = (Driver) Class.forName(JDBC_DRIVER).newInstance();
			DriverManager.registerDriver(d);
		} catch(Exception e){
			Logger.log("Error loading database driver: " + e.toString());
			return;
		}
		try{
			conn = DriverManager.getConnection(DB_URL + DB_FILE, USER, PASS);
			try(PreparedStatement create = conn
					.prepareStatement("CREATE TABLE IF NOT EXISTS log (aid VARCHAR(100), message VARCHAR(10000))")){
				create.execute();
			}
		} catch(SQLException e){
			Logger.log(e);
		}
	}

	/**
	 * @param msg
	 */
	public void insert(List<LogMessage> msgs){
		try{
			try(PreparedStatement insert = conn
					.prepareStatement("INSERT INTO LOG (aid, message) VALUES (?, ?)")){
				for(LogMessage msg : msgs){
					insert.setString(1, msg.getSender().toString());
					insert.setString(2, msg.getString());
					insert.execute();
				}
			}
		} catch(SQLException e){
			Logger.log(e);
		}
	}

	/**
	 * @param logMessage
	 */
	public void insert(LogMessage msg){
		try{
			try(PreparedStatement insert = conn
					.prepareStatement("INSERT INTO LOG (aid, message) VALUES (?, ?)")){
					insert.setString(1, msg.getSender().toString());
					insert.setString(2, msg.getString());
					insert.execute();
			}
		} catch(SQLException e){
			Logger.log(e);
		}
		
	}
}
