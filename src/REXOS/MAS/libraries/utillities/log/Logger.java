/**
 * @file src/REXOS/MAS/libraries/utillities/log/Logger.java
 * @brief Helper for log messages, providing a single point for controlling program origin.
 * @date Created: 17 mei 2013
 *
 * @author Jan-Willem Willebrands
 * @author Alexander Streng
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
 **/
package libraries.utillities.log;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;

import org.apache.commons.lang.StringUtils;

import libraries.knowledgedb_client.Row;

/**
 * Helper for log messages, providing a single point for controlling program origin.
 **/
public class Logger {
	protected static final Character bufferCharacter = ' ';
	protected static final SimpleDateFormat fileDateFormat = new SimpleDateFormat("YY-MM-dd HH:mm:ss");
	protected static final SimpleDateFormat outputDateFormat = new SimpleDateFormat("YY-MM-dd HH:mm:ss");
	protected static final long relativeTimeReferenceTime = Calendar.getInstance().getTimeInMillis();
	protected static final boolean useRelativeTime = false;
	
	protected static FileWriter logFileFileWriter = null;
	protected static BufferedWriter logFileBufferedWriter = null;

	protected static final File logFile = new File("log.txt");
	
	
	private static final boolean isPrintToConsoleEnabled = true;
	private static final boolean isPrintToLogFileEnabled = false;

	/**
	 * @var boolean testingEnabled
	 * Controls wether or not logging of test messages is enabled
	 */
	private static final boolean testingEnabled = true;
	private static String TEST_DATA_DIR = "";
	
    private static final String PATH_ENVIRONMENT_VARIABLE = "MSGPATH";
	
	/**
	 * @var int logleveltreshhold
	 * treshhold for showing log msg
	 **/
	public static final int loglevelThreshold = LogLevel.DEBUG.getLevel();

	public static final boolean logToFileEnabled = false;

	static {
		String msgsFilePath = System.getenv(PATH_ENVIRONMENT_VARIABLE);
		if (msgsFilePath != null){
			File dir = new File (msgsFilePath);
			if(dir.exists()) {
				System.out.println("Log Directory detected - Removing old log files");
				String[] files = dir.list();
				
				for(String filename : files) {
						File file = new File(filename);
						if (file.exists()){	
							file.delete();					
						}
				}
			}
			else{
				dir.mkdirs();
			}
		}
	}
	



	public static boolean eraseLogFile(){
		if(logFile.exists())	return logFile.delete();
		else					return false;
	}

	
	
	
	
	
	
	
	
	
	
	
	public static void log(LogLevel logLevel, String message, Object object) {
		handleLogEntry(LogSection.NONE, logLevel, message, new Object[] {} );
	}
	public static void log(LogLevel logLevel, String message, Object[] objects) {
		handleLogEntry(LogSection.NONE, logLevel, message, objects);
	}
	public static void log(LogSection logSection, LogLevel logLevel, String message) {
		handleLogEntry(logSection, logLevel, message, new Object[] {} );
	}	
	public static void log(LogSection logSection, LogLevel logLevel, String message, Object object) {
		handleLogEntry(logSection, logLevel, message, new Object[] {object} );
	}
	public static void log(LogSection logSection, LogLevel logLevel, String message, Object[] objects) {
		handleLogEntry(logSection, logLevel, message, objects);
	}
	
	protected static void handleLogEntry(LogSection logSection, LogLevel logLevel, String message, Object[] objects) {
		String serializedLogEntry = serializeLogEntry(logSection, logLevel, message, objects);
		if(isPrintToConsoleEnabled == true) {
			printToOut(logLevel, serializedLogEntry);
		}
		if(isPrintToLogFileEnabled == true) {
			printToFile(serializedLogEntry);
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	protected static int calulateMaxLengthOfLogSection() {
		int maxLength = 0;
		for (LogSection logSection : LogSection.values()) {
			if(logSection.getName().length() > maxLength) {
				maxLength = logSection.getName().length();
			}
		}
		return maxLength;
	}
	protected static int calulateMaxLengthOfLogLevel() {
		int maxLength = 0;
		for (LogLevel logLevel : LogLevel.values()) {
			if(logLevel.getName().length() > maxLength) {
				maxLength = logLevel.getName().length();
			}
		}
		return maxLength;
	}
	
	protected static String serializeLogEntry(LogSection logSection, LogLevel logLevel, String message, Object[] objects) {
		int maxLogSectionLength = calulateMaxLengthOfLogSection();
		int maxLogLevelLength = calulateMaxLengthOfLogLevel();
		
		// write the logLevel en logSection segments for the output and stuff them with spaces so that all the outputs are vertically aligned
		int currentlogLevelLength = logLevel.getName().length();
		String logLevelSegment = logLevel.getName() + StringUtils.repeat(String.valueOf(bufferCharacter), maxLogLevelLength - currentlogLevelLength);
		int currentlogSectionLength = logSection.getName().length();
		String logSectionSegment = logSection.getName() + StringUtils.repeat(String.valueOf(bufferCharacter), maxLogSectionLength - currentlogSectionLength);
		
		String logLevelAndSectionSegment = "[" + logLevelSegment + "] [" + logSectionSegment + "] "; 
		
		if(message == null) {
			throw new RuntimeException("Unable to serialize a logEntry which has no message");
		} else if(objects == null) {
			throw new RuntimeException("Unable to serialize a logEntry if objects is null (instead use an empty array)");
		}
		
		if(objects.length != 0) {
			String output = logLevelAndSectionSegment + message + "\n";
			for (Object object : objects) {
				// stuff every line so that all the outputs are vertically aligned
				output += StringUtils.repeat(String.valueOf(bufferCharacter), logLevelAndSectionSegment.length()) + object.toString() + "\n";
			}
			return output;
		} else {
			String output = logLevelAndSectionSegment + message;
			return output;
		}
	}
	
	private static void printToOut(LogLevel logLevel, String serializedLogEntry)
	{
		switch(logLevel) {		
			case EMERGENCY:					
			case ALERT:
			case CRITICAL:
			case ERROR:
				System.err.println(serializedLogEntry);
				break;
			case WARNING:
			case NOTIFICATION:
			case INFORMATION:
			case DEBUG:
				System.out.println(serializedLogEntry);
				break;
		}
	}
	/** 
	 * Writes the specified message to the gridLog.txt using the BufferedWriter and FileWriter, also adding the class the Logger was called from.
	 * @param level The LoggerLevel of the message
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param hasThrowable PrintToOut already looks for a Throwable, it sets a boolean 
	 **/
	private static void printToFile(String serializedLogEntry){
		//Set date format
		String timeLog = fileDateFormat.format(Calendar.getInstance().getTime());
		
		if(logFileFileWriter == null) {
			// first time we write a entry to the logFile, so we need to create the writers
			try {
				if(logFile.exists() == false) {
					logFile.createNewFile();
				}
				logFileFileWriter = new FileWriter(logFile, true);
				logFileBufferedWriter = new BufferedWriter(logFileFileWriter);
				
				logFileBufferedWriter.write(serializedLogEntry);
				
				
			} catch (IOException ex) {
				System.err.println("Unable to write to logFile:");
				ex.printStackTrace();
			} finally {
				try {
					//Close the writer regardless of what happens...
					logFileBufferedWriter.close();
					logFileFileWriter.close();
				} catch (Exception ex) {
					//Well ... we tried ...
				}
			}
		}
	}
}