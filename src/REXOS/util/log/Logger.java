/**
 * @file src/REXOS/MAS/libraries/log/Logger.java
 * @brief Helper for log messages, providing a single point for controlling program origin.
 * @date Created: 17 mei 2013
 *
 * @author Jan-Willem Willebrands
 * @author Alexander Streng
 * @author Tommas Bakker
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
package util.log;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import org.apache.commons.lang.ArrayUtils;
import org.apache.commons.lang.StringUtils;

/**
 * Helper for log messages, providing a single point for controlling program origin.
 **/
public class Logger {
	enum LogSectionSelection {
		INCLUDED,
		EXCLUDED,
		UNDEFINED;
	}
	
	protected static final Character stuffCharacter = ' ';
	protected static final String logFileSeperator = "\n" + 
			"##############################################################################" + 
			"##############################################################################\n" + 
			"##############################################################################" + 
			"##############################################################################\n" + 
			"##############################################################################" + 
			"##############################################################################\n\n";
	protected static final SimpleDateFormat dateFormat = new SimpleDateFormat("YYYY-MM-dd HH:mm:ss.SSS");
	protected static final long relativeTimeReferenceTime = Calendar.getInstance().getTimeInMillis();
	
	protected static FileWriter logFileFileWriter = null;
	protected static BufferedWriter logFileBufferedWriter = null;
	protected static final File logFile = new File("log.txt");
	
	protected static boolean isPrintToConsoleEnabled = true;
	protected static boolean isPrintToLogFileEnabled = true;
	protected static boolean printLogSectionsWithUndefinedSelections = true;
	protected static boolean eraseLogFileOnStartup = false;
	protected static boolean seperateStartupInLogFile = true;
	
	protected static boolean useRelativeTime = false;
	protected static boolean logTime = true;
	protected static boolean logLogSection = true;
	protected static boolean logLogLevel = true;
	protected static boolean logCallingMethod = true;
	protected static boolean logMessage = true;
	protected static boolean logStacktrace = true;
	protected static boolean logObjects = true;
	
	
	protected static LogSection[] includedLogSections = new LogSection[] {LogSection.NONE, LogSection.HAL};
	protected static LogSection[] excludedLogSections = new LogSection[] {LogSection.HAL_MODULE_FACTORY_SQL, LogSection.HAL_CAPABILITY_FACTORY_SQL};
	protected static LogLevel loglevelThreshold = LogLevel.DEBUG;
	
	
	public static void log(String message) {
		handleLogEntry(LogSection.NONE, LogLevel.DEBUG, message, new Object[] {} );
	}
	public static void log(String message, Object object) {
		handleLogEntry(LogSection.NONE, LogLevel.DEBUG, message, new Object[] {object} );
	}
	public static void log(LogLevel logLevel, String message, Object object) {
		handleLogEntry(LogSection.NONE, logLevel, message, new Object[] {object} );
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
		boolean logLevelAboveThreshold = calculateLogLevelAboveThreshold(logLevel);
		LogSectionSelection selection = calculateLogSectionSelection(logSection);
		
		if(logLevelAboveThreshold == true && (selection == LogSectionSelection.INCLUDED || 
				(selection == LogSectionSelection.UNDEFINED && printLogSectionsWithUndefinedSelections == true)
				)) {
			String serializedLogEntry = serializeLogEntry(logSection, logLevel, message, objects);
			if(isPrintToConsoleEnabled == true) {
				printToOut(logLevel, serializedLogEntry);
			}
			if(isPrintToLogFileEnabled == true) {
				printToFile(serializedLogEntry);
			}
		}
	}
	private static boolean calculateLogLevelAboveThreshold(LogLevel logLevel) {
		if(logLevel.getLevel() >= loglevelThreshold.getLevel()) return true;
		else return false;
	}
	private static LogSectionSelection calculateLogSectionSelection(LogSection logSection) {
		LogSection currentLogSection = logSection;
		// search the tree of logSections to determine the selection of the logSection
		while(currentLogSection != null) {
			if(ArrayUtils.contains(includedLogSections, currentLogSection) == true) return LogSectionSelection.INCLUDED;
			if(ArrayUtils.contains(excludedLogSections, currentLogSection) == true) return LogSectionSelection.EXCLUDED;
			currentLogSection = currentLogSection.getParentSection();
		}
		return LogSectionSelection.UNDEFINED;
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
		String output = new String();
		boolean newLineRequired = false;
		
		if(message == null) {
			throw new RuntimeException("Unable to serialize a logEntry which has no message");
		}
		if(objects == null) {
			throw new RuntimeException("Unable to serialize a logEntry if objects is null (instead use an empty array)");
		}
		
		// write the logLevel and logSection segments for the output and stuff them with spaces so that all the outputs are vertically aligned
		if(logLogLevel == true) {
			int maxLogLevelLength = calulateMaxLengthOfLogLevel();
			int currentlogLevelLength = logLevel.getName().length();
			String logLevelSegment = logLevel.getName() + StringUtils.repeat(String.valueOf(stuffCharacter), maxLogLevelLength - currentlogLevelLength);
			output += "[" + logLevelSegment + "] ";
		}
		if(logLogSection == true) {
			int maxLogSectionLength = calulateMaxLengthOfLogSection();
			int currentlogSectionLength = logSection.getName().length();
			String logSectionSegment = logSection.getName() + StringUtils.repeat(String.valueOf(stuffCharacter), maxLogSectionLength - currentlogSectionLength);
			output += "[" + logSectionSegment + "] ";
		}
		
		// the number of bufferCharacters to align the following lines behind the logLevel and logSection
		int numberOfRequiredStuffCharactersForNewLine = output.length();
		
		if(logTime == true) {
			if(useRelativeTime == true) {
				long currentTime = Calendar.getInstance().getTimeInMillis();
				long relativeTime = currentTime - relativeTimeReferenceTime;
				
				output += String.valueOf(relativeTime) + " ";
			} else {
				String currentTime = dateFormat.format(Calendar.getInstance().getTime());
				output += currentTime + " ";
				
			}
		}
		
		
		if(logCallingMethod == true) {
			// get the calling method
			String className = 	Thread.currentThread().getStackTrace()[4].getClassName();
			String methodName = Thread.currentThread().getStackTrace()[4].getMethodName();
			String fileName = 	Thread.currentThread().getStackTrace()[4].getFileName();
			int lineNumber = 	Thread.currentThread().getStackTrace()[4].getLineNumber();
			String sourceSegment = "@ " + className + "." + methodName + "(" + fileName + ":" + lineNumber + ")";
			output += sourceSegment;
			// a new line is required, but we also have to make sure that stuffing is done if needed. So set the newLineRequired
			newLineRequired = true;
		}
		
		if(logMessage == true) {
			if(newLineRequired == true) {
				output += "\n" + generateStuffedOffset(numberOfRequiredStuffCharactersForNewLine);
			}
			output += message;
			// a new line is required, but we also have to make sure that stuffing is done if needed. So set the newLineRequired
			newLineRequired = true;
		}
		
		if(logObjects == true) {
			for (Object object : objects) {
				if(newLineRequired == true) {
					output += "\n" + generateStuffedOffset(numberOfRequiredStuffCharactersForNewLine);
				}
				
				String objectSegment = new String();
				if(object instanceof Throwable) {
					Throwable throwable = (Throwable)object;
					ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
					PrintStream printStream = new PrintStream(byteArrayOutputStream);
					throwable.printStackTrace(printStream);

					try {
						objectSegment += byteArrayOutputStream.toString("UTF8") + "\n";
					} catch (UnsupportedEncodingException ex) {
						// as the stacktrace is in latin, this should never happen.
						ex.printStackTrace();
					}
				} else {
					objectSegment += object.toString();
				}
				
				// stuff every line so that all the outputs are vertically aligned
				output += stuffEveryNewLine(objectSegment, numberOfRequiredStuffCharactersForNewLine);
				// a new line is required, but we also have to make sure that stuffing is done if needed. So set the newLineRequired
				newLineRequired = true;
			}
			
			return output;
		}
		return output;
	}
	
	private static String stuffEveryNewLine(String objectSegment, int numberOfRequiredStuffCharactersForNewLine) {
		String[] lines = objectSegment.split("\n");
		String output = new String();
		for(int i = 0; i < lines.length; i++) {
			if(i == 0) {
				// no need to stuff here
				output += lines[i] + "\n";
			} else {
				output += generateStuffedOffset(numberOfRequiredStuffCharactersForNewLine) + lines[i] + "\n";
			}
		}
		return output;
	}
	private static String generateStuffedOffset(int numberOfRequiredStuffCharactersForNewLine) {
		return StringUtils.repeat(String.valueOf(stuffCharacter), numberOfRequiredStuffCharactersForNewLine);
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
	 * Writes the serialized logEntry message to the logFile using the BufferedWriter and FileWriter.
	 * If the logFile was not yet created, it will be created.
	 **/
	private static void printToFile(String serializedLogEntry){
		try {
			if(logFileFileWriter == null) {
				// first time we write a entry to the logFile, so we need to create the writers
				if(logFile.exists() == false) {
					logFile.createNewFile();
				} else if(eraseLogFileOnStartup == true) {
					eraseLogFile();
				} else if(seperateStartupInLogFile == true) {
					logFileFileWriter = new FileWriter(logFile, true);
					logFileBufferedWriter = new BufferedWriter(logFileFileWriter);
					logFileBufferedWriter.write(logFileSeperator);
				} else {
					logFileFileWriter = new FileWriter(logFile, true);
					logFileBufferedWriter = new BufferedWriter(logFileFileWriter);
				}
			}
			logFileBufferedWriter.write(serializedLogEntry);
			// mimic println by adding a new line at the end of every logEntry
			logFileBufferedWriter.write("\n");
			
			// always flush the streams to make sure that the content has been written
			logFileBufferedWriter.flush();
			logFileFileWriter.flush();
		} catch (IOException ex) {
			System.err.println("Logger was unable to write to logFile:");
			ex.printStackTrace();
			
			try {
				logFileBufferedWriter.close();
				logFileFileWriter.close();
			} catch (Exception ex2) {
				System.err.println("Logger was unable to close the logFile:");
				ex2.printStackTrace();
			}
		}
	}
	public static boolean eraseLogFile(){
		if(logFile.exists()) return logFile.delete();
		else return false;
	}
	
}