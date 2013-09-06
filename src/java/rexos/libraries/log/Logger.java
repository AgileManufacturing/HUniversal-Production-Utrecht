/**
 * @file rexos/libraries/log/Logger.java
 * @brief Helper for log messages, providing a single point for controlling program output.
 * @date Created: 17 mei 2013
 *
 * @author Jan-Willem Willebrands
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
package rexos.libraries.log;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;

import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.mas.data.LogLevel;

/**
 * Helper for log messages, providing a single point for controlling program output.
 **/
public class Logger {
	/**
	 * @var PrintStream errStream
	 * Stream that will be used for writing error messages, i.e. throwables.
	 **/
	private static final PrintStream errStream = System.err;
	
	/**
	 * @var PrintStream outStream
	 * Stream that will be used for writing log messages.
	 **/
	private static final PrintStream outStream = System.out;
	
	/**
	 * @var boolean debugEnabled
	 * Controls whether or not printing of log messages is enabled.
	 **/
	private static final boolean debugEnabled = true;
	
    private static final String PATH_ENVIROMENT_VARIABLE = "MSGPATH";
	
	/**
	 * @var int logleveltreshhold
	 * treshhold for showing log msg
	 **/
	
	public static final int loglevelThreshold = LogLevel.DEBUG.getLevel();
	/**
	 * Returns whether or not debugging is enabled.
	 * @return true if debugging is enabled, false otherwise.
	 **/
	public static boolean isDebugEnabled() {
		return debugEnabled;
	}
	
	/**
	 * Writes the String representation (as returned by obj.toString) of the given object to the output stream.
	 * @param obj The Object that should be printed.
	 **/
	public static void log(LogLevel level, Object obj) {
		printToOut(level, obj.toString());
	}
	
	/**
	 * Writes the specified message to the output stream.
	 * @param msg The message that should be printed.
	 **/
	public static void log(LogLevel level, String msg) {
		printToOut(level, msg);
	}
	
	/**
	 * Writes the specified message to the output stream using the PrintStream format method.
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param objects Arguments referenced by the format specifiers in the format string.
	 **/
	public static void log(LogLevel level, String msg, Object... objects) {
		printToOut(level, String.format(msg, objects));
	}
	
	public static void logAclMessage(ACLMessage msg) 
	{
        String msgsFilePath = System.getenv(PATH_ENVIROMENT_VARIABLE);
        
    	try { 
    		File dir = new File (msgsFilePath);
    		if (!dir.exists()){
    			dir.mkdir();
    		}
    		File file = new File(dir, msg.getConversationId() + ".txt");
    		
    		//if file doesnt exists, then create it
    		if(!file.exists()){
    			file.createNewFile();
    		}    		
    		FileWriter fileWritter = new FileWriter(file, true);
	        BufferedWriter bufferWritter = new BufferedWriter(fileWritter);
	        
	        bufferWritter.write("Msg from ( sender ): " + msg.getSender().getLocalName() + " -> " + "to ( receiver ): " + ((AID)msg.getAllReceiver().next()).getLocalName() +  "\n" +
	        		"Performative: " + ACLMessage.getPerformative(msg.getPerformative()) + "\n" +
	        		"Ontology: " + msg.getOntology()  + "\n\n");
	        
	        bufferWritter.close();
	        
    	}catch(IOException e){
    		e.printStackTrace();
    	}
	}
	
	/**
	 * Writes the specified message to the output stream using the PrintStream format method.
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param objects Arguments referenced by the format specifiers in the format string.
	 **/
	public static void log(LogLevel level, Throwable throwable) {
		printToOut(level, throwable.getStackTrace().toString());
	}
	
	
	private static void printToOut(LogLevel level, String msg)
	{
		if(level.getLevel() >= loglevelThreshold){
			switch(level){		
				case EMERGENCY:
					System.err.println("EMERGENCY:\t" + msg + "\nAt " + new Exception().getStackTrace()[2]);
					break;
					
				case ALERT:
					System.err.println("ALERT:\t" + msg + msg + "\nAt " + new Exception().getStackTrace()[2]);
					break;
					
				case CRITICAL:
					System.err.println("CRITICAL:\t" + msg + "\nAt " + new Exception().getStackTrace()[2]);
					break;
					
				case ERROR:
					System.err.println("ERROR:\t" + msg + "\nAt " + new Exception().getStackTrace()[2]);
					break;
					
				case WARNING:
					System.out.println("WARNING:\t" + msg);
					break;
					
				case NOTIFICATION:
					System.out.println("NOTIFICATION:\t" + msg);
					break;
					
				case INFORMATION:
					System.out.println("INFORMATION:\t" + msg);
					break;
					
				case DEBUG:
					System.out.println("DEBUG:\t" + msg);
					break;
					
			}
		}
	}

}
