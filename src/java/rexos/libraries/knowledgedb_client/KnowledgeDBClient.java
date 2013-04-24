/**
 * @author Ammar Abdulamir
 * @author Arjen van Zanten
 * @author Jan-Willem Willebrands
 * @file KnowledgeDBClient.java
 * @brief A client to communicate with knowledge database.
 * @date Created: 2013-04-05
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
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
 **/
package rexos.libraries.knowledgedb_client;

import com.mysql.jdbc.Connection;
import com.mysql.jdbc.JDBC4PreparedStatement;

import java.io.FileInputStream;
import java.io.IOException;
import java.sql.*;
import java.util.ArrayList;
import java.util.Properties;

/**
 * A client to communicate with knowledge database.
 **/
public class KnowledgeDBClient {
    /**
     * @var String PROPERTIES_ENVIRONMENT_VARIABLE
     *
     * The environment variable that holds the path to the properties file.
     **/
    private static final String PROPERTIES_ENVIRONMENT_VARIABLE = "KNOWLEDGE_DB_PROPERTIES";

    /**
     * @var rexos.libraries.knowledgedb_client.KnowledgeDBClient client
     *
     * The only rexos.libraries.knowledgedb_client.KnowledgeDBClient instance.
     **/
    private static KnowledgeDBClient client;

    /**
     * @var Connection connection
     *
     * The mysql connection.
     **/
    private Connection connection;

    /**
     * Get current rexos.libraries.knowledgedb_client.KnowledgeDBClient instance.
     *
     * @return The current rexos.libraries.knowledgedb_client.KnowledgeDBClient.
     * @throws KnowledgeException Creating a connection to the knowledge database has failed.
     **/
    public static synchronized KnowledgeDBClient getClient() throws KnowledgeException {
        if (client == null) {
            client = new KnowledgeDBClient();
        }

        return client;
    }

    /**
     * Private constructor to create a mysql connection.
     * @throws KnowledgeException Creating a connection to the knowledge database has failed.
     **/
    private KnowledgeDBClient() throws KnowledgeException {
        try {
            Properties dbProperties = new Properties();
            FileInputStream in = new FileInputStream(System.getenv(PROPERTIES_ENVIRONMENT_VARIABLE));
            dbProperties.load(in);

            String url = "jdbc:mysql://" + dbProperties.getProperty("host") + ":" + dbProperties.getProperty("port")
                    + "/" + dbProperties.getProperty("db");
            in.close();

            connection = (Connection) DriverManager.getConnection(url, dbProperties.getProperty("username"), dbProperties.getProperty("password"));
        } catch (SQLException ex) {
            throw new KnowledgeException("Failed to connect to the knowledge server.", ex);
        } catch (IOException ex) {
            throw new KnowledgeException("Failed to read from the properties file.", ex);
        }
    }

    /**
     * Creates an array of rows from the given ResultSet.
     * @param result The ResultSet that needs to be converted.
     * @return A Row array containing every row from the ResultSet.
     * @throws SQLException Reading from the ResultSet failed.
     **/
	private Row[] createRowArrayFromResultSet(ResultSet result) throws SQLException {
		ArrayList<Row> rowList = new ArrayList<Row>();
		while (result.next()) {
			rowList.add(new Row(result));
		}

		Row[] rows = new Row[rowList.size()];
		rowList.toArray(rows);
		return rows;
	}
    
    /**
     * Get column names in a string array.
     * @param resultSet The ResultSet from a statement execution.
     *
     * @return A string array of column names.
     **/
    public String[] getColumns(ResultSet resultSet) {
        ArrayList<String> columns = new ArrayList<String>();

        try {
            ResultSetMetaData metadata = resultSet.getMetaData();

            for (int i = 1; i <= metadata.getColumnCount(); i++) {
                columns.add(metadata.getColumnLabel(i));
            }
        } catch (SQLException e) {
            e.printStackTrace();
        }

        String[] ret = new String[columns.size()];
        columns.toArray(ret);

        return ret;
    }

    /**
     * Executes a single query statement.
     *
     * @param query The query to be executed.
     *
     * @return A ResultSet objected generated by the query.
     * @throws KnowledgeException Reading from the knowledge database failed.
     **/
    public Row[] executeSelectQuery(String query) throws KnowledgeException {
    	Statement statement = null;
    	ResultSet result = null;
    	Row[] rows = null;
    	try {
	        statement = connection.createStatement();
	        result = statement.executeQuery(query);
	        rows = createRowArrayFromResultSet(result);
    	} catch (SQLException ex) {
    		throw new KnowledgeException("Error reading from the knowledge database.", ex);
    	} finally {
    		if (result != null)	try { result.close(); } catch (SQLException e) {}
    		if (statement != null)	try { statement.close(); } catch (SQLException e) {}
    	}
        
    	return rows;
    }

    /**
     * Executes a single query statement with parameters.
     *
     * @param query The query to be executed.
     * @param parameters The parameters for the query in a consecutive order.
     *
     * @return A ResultSet objected generated by the query.
     * @throws KnowledgeException Reading from the knowledge database failed.
     **/
    public Row[] executeSelectQuery(String query, Object... parameters) throws KnowledgeException {
        PreparedStatement statement = null;
        ResultSet result = null;
        Row[] rows = null;
    	try {
			statement = connection.prepareStatement(query);

			for (int i = 0; i < parameters.length; i++) {
			    statement.setString(i + 1, parameters[i].toString());
			}

			result = statement.executeQuery();
	        rows = createRowArrayFromResultSet(result);
		} catch (SQLException ex) {
    		throw new KnowledgeException("Error reading from the knowledge database.", ex);
		} finally {
			if (result != null)	try { result.close(); } catch (SQLException e) {}
    		if (statement != null)	try { statement.close(); } catch (SQLException e) {}
		}
    	
    	return rows;
    }


    /**
     * Executes an insert or update query.
     *
     * @param query The insert or update query.
     *
     * @return Last insert ID on successful insert query, or 0 for an update query.
     * @throws KnowledgeException Reading from the knowledge database failed.
     **/
    public int executeUpdateQuery(String query) throws KnowledgeException{
        return executeUpdateQuery(query, null);
    }

    /**
     * Executes an insert or update query.
     *
     * @param query The insert or update query.
     * @param parameters The parameters for the query in a consecutive order.
     *
     * @return Last insert ID on successful insert query, or 0 for an update query.
     * @throws KnowledgeException Reading from the knowledge database failed.
     **/
    public int executeUpdateQuery(String query, Object[] parameters) throws KnowledgeException {
    	PreparedStatement statement = null;
    	ResultSet result = null;
    	int queryReturnValue = 0;
        try {
			statement = connection.prepareStatement(query, Statement.RETURN_GENERATED_KEYS);

			if (parameters != null) {
			    for (int i = 0; i < parameters.length; i++) {
			        statement.setString(i + 1, parameters[i].toString());
			    }
			}

			statement.executeUpdate();
			if (((JDBC4PreparedStatement) statement).getLastInsertID() > 0) {
			    result = statement.getGeneratedKeys();
			    result.next();
			    queryReturnValue = result.getInt(1);
			} else {
				queryReturnValue = 0;
			}
		} catch (SQLException ex) {
			throw new KnowledgeException("Error reading from the knowledge database.", ex);
		} finally {
			if (result != null)	try { result.close(); } catch (SQLException e) {}
    		if (statement != null)	try { statement.close(); } catch (SQLException e) {}
		}
        
        return queryReturnValue;
    }
}