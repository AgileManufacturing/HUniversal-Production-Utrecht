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

/**
 * Helper for log messages, providing a single point for controlling program origin.
 **/
public class Logger {	
	/**
	 * @var boolean debugEnabled
	 * Controls whether or not printing of log messages is enabled.
	 **/
	private static final boolean debugEnabled = true;

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

	public static final File logFile = new File("log.txt");
	
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
	
	/**
	 * Returns whether or not debugging is enabled.
	 * @return true if debugging is enabled, false otherwise.
	 **/
	public static boolean isDebugEnabled() {
		return debugEnabled;
	}

	/**
	 * Returns whether or not debugging is enabled.
	 * @return true if debugging is enabled, false otherwise.
	 **/
	public static boolean islogToFileEnabled() {
		return logToFileEnabled;
	}

	/**
	 * Writes the String representation (as returned by obj.toString) of the given object to the origin stream.
	 * @param obj The Object that should be printed.
	 **/
	public static void log(LogLevel level, Object obj) {
		printToOut(level, obj.toString());
	}

	/**
	 * Writes the specified message to the origin stream.
	 * @param msg The message that should be printed.
	 **/
	public static void log(LogLevel level, String msg) {
		printToOut(level, msg);
	}

	/**
	 * Writes the specified message to the origin stream using the PrintStream format method.
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param objects Arguments referenced by the format specifiers in the format string.
	 **/
	public static void log(LogLevel level, String msg, Object... objects) {
		printToOut(level, String.format(msg, objects));
	}
	
	public static void log(LogLevel level, String msg,  Throwable throwable) {
		printToOut(level, msg);
		printToOut(level, throwable.getStackTrace().toString());
	}


	public static void log(LogLevel level, Throwable throwable) {
		printToOut(level, throwable.getStackTrace().toString());
	}

	/** 
	 * Writes the specified message to the origin stream using the PrintStream format method.
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param objects Arguments referenced by the format specifiers in the format string.
	 **/
	private static void printToOut(LogLevel level, Object msg)
	{
		if(level.getLevel() >= loglevelThreshold){
			switch(level){		

			case EMERGENCY:					
			case ALERT:
			case CRITICAL:
			case ERROR:

				if(msg.getClass() == Throwable.class){
					System.err.println(level.name() + "\t" + ((Throwable)msg).getStackTrace().toString());
					if(logToFileEnabled)	printToFile(level, msg, true);
				}
				else{
					System.err.println("ERROR:\t" + msg);
					if(logToFileEnabled)	printToFile(level, msg, false);
				}
				break;

			case WARNING:
			case NOTIFICATION:
			case INFORMATION:
			case DEBUG:
				System.out.println(level.name() + "\t" + msg);
				if(logToFileEnabled)		printToFile(level, msg, false);
				break;

			}
		}
	}

	/** 
	 * Writes the specified message to the gridLog.txt using the BufferedWriter and FileWriter, also adding the class the Logger was called from.
	 * @param level The LoggerLevel of the message
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param hasThrowable PrintToOut already looks for a Throwable, it sets a boolean 
	 **/
	private static void printToFile(LogLevel level, Object msg, boolean hasThrowable){
		//Set date format
		String timeLog = new SimpleDateFormat("MM-dd HH:mm:ss").format(Calendar.getInstance().getTime());

		//when logToFileEnabled is true write to gridLog.txt
		if(logToFileEnabled){
			
			BufferedWriter writer = null;
			try
			{
				if(logFile.exists()){
					//if the file exists write to it, not overwriting old data.
					writer = new BufferedWriter(new FileWriter(logFile, true));
					
					//logs date, time and LoggerLevel
					writer.write("[" + timeLog + "][" + level + "]\t");
					
					//if the LogLevel is DEBUG, ERROR or ALERT we add an extra indention for more structure
					if(level == LogLevel.DEBUG || level == LogLevel.ALERT || level == LogLevel.ERROR){
						writer.write("\t");
					}

					//track from where the Logger has been called, and add it to the gridLog.txt
					String[] stack = Thread.currentThread().getStackTrace()[4].getClassName().split("\\.");
					String origin = "";
					
					//if this segment of stack equals to "behaviours" you should print an additional part of the path
					if(stack[stack.length-1].equals("behaviours")){
						origin = "@" + stack[stack.length-3] + "/" + stack[stack.length-2] + "/" + stack[stack.length-1] + ":";
					} else {
						origin = "@" + stack[stack.length-2] + "/" + stack[stack.length-1] + ":";
					}				

					//to outline the gridLog.txt we want to add different amounts of tabs according to the length of the origin
					if(origin.length() < 24){
						origin += "\t\t\t";
					} else if(origin.length() < 32){
						origin += "\t\t";
					} else if(origin.length() < 40){
						origin += "\t";
					}

					writer.write(origin);
					
					if(hasThrowable)					writer.write(((Throwable)msg).getStackTrace().toString());
					else								writer.write(msg.toString());

					//if the message already ends with a line break we shouldn't add a new one
					//if(!msg.toString().endsWith("\n"))	writer.write("\n");
					writer.write((msg.toString().replace("%n", "; ")).replace("\n", "; ") + "\n");
					
					//close the writer
					writer.close();
				} else {
					//if the file doesn't exist create it
					logFile.createNewFile();
				}
			} catch(Exception e) {
				e.printStackTrace();
			} finally {
				try {
					//Close the writer regardless of what happens...
					writer.close();
				} catch (Exception e) {
					//Well ... we tried ...
				}
			}
		}
	}

	public static boolean eraseLogFile(){
		if(logFile.exists())	return logFile.delete();
		else					return false;
	}
}