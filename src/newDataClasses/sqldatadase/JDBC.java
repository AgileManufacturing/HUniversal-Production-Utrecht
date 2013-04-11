/**
 * 
 */
package newDataClasses.sqldatadase;

import java.sql.Connection;
import java.sql.Driver;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;

/**
 * @author Theodoor http://www.tutorialspoint.com/jdbc/jdbc-create-database.htm
 * 
 */
public class JDBC {
	// JDBC driver name and database URL
	static final String JDBC_DRIVER = "org.sqlite.JDBC";
	static final String DB_URL = "jdbc:sqlite:test.db";

	// Database credentials
	static final String USER = "username";
	static final String PASS = "password";
	
	public static void main(String[] args) {
		Connection conn = null;
		Statement stmt = null;
		try {
			// STEP 1: Setup the Driver
			try {
				// Load the JDBC driver class dynamically.
				Driver d = (Driver) Class.forName(JDBC_DRIVER).newInstance();
				DriverManager.registerDriver(d);
			} catch (Exception e) {
				System.out.println("Error loading database driver: "
						+ e.toString());
				return;
			}

			// STEP 3: Open a connection
			System.out.println("Connecting to database...");
			conn = DriverManager.getConnection(DB_URL, USER, PASS);

			
			// STEP 4: Execute a query
			System.out.println("Creating database...");
			stmt = conn.createStatement();

		//	String sql = "CREATE DATABASE STUDENTS";
		//	stmt.executeUpdate(sql);
			System.out.println("Database created successfully...");
		} catch (SQLException se) {
			// Handle errors for JDBC
			se.printStackTrace();
		} catch (Exception e) {
			// Handle errors for Class.forName
			e.printStackTrace();
		} finally {
			// finally block used to close resources
			try {
				if (stmt != null)
					stmt.close();
			} catch (SQLException se2) {
				
			}
			try {
				if (conn != null)
					conn.close();
			} catch (SQLException se) {
				se.printStackTrace();
			}
		}
		System.out.println("Goodbye!");
	}
}