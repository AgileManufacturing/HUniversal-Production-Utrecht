/**
 * @author Ammar Abdulamir
 * @file Row.java
 * @brief A class representing a database row.
 * @date Created: 2013-04-08
 * @section LICENSE
 * License: newBSD
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
 **/
package rexos.libraries.knowledgedb_client;

import java.sql.ResultSet;
import java.sql.ResultSetMetaData;
import java.sql.SQLException;
import java.util.HashMap;
import java.util.Map;

/**
 * A class representing a database row.
 **/
public class Row {
    /**
     * @var HashMap<String, Object> row
     *
     * A hashmap containing the row data with column name as keys and values as object.
     **/
    private final HashMap<String, Object> row;

    /**
     * A constructor for constructing a row from a ResultSet.
     *
     * @param resultSet The ResultSet from a statement execution.
     * @throws SQLException Failed to read from the ResultSet.
     **/
    public Row(ResultSet resultSet) throws SQLException {
        row = new HashMap<String, Object>();

        ResultSetMetaData metadata = resultSet.getMetaData();
        for (int i = 1; i <= metadata.getColumnCount(); i++) {
            row.put(metadata.getColumnLabel(i), resultSet.getObject(i));
        }
    }

    /**
     * Get value from specific column.
     *
     * @param column The name of the column.
     *
     * @return The value of the column.
     **/
    public Object get(String column) throws KeyNotFoundException {
        if (!row.containsKey(column)) {
            throw new KeyNotFoundException("No column with name " + column + " was found in this row.");
        }

        return row.get(column);
    }


    /**
     * Convert a row to a string representative.
     *
     * @return A string that represents a row.
     **/
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder(Row.class.getName());
        builder.append("{ ");
        for (Map.Entry<String, Object> entry : row.entrySet()) {
            builder.append(entry.getKey());
            builder.append('=');
            builder.append('"');
            builder.append(entry.getValue());
            builder.append("\" ");
        }
        builder.append("}");

        return builder.toString();
    }
}