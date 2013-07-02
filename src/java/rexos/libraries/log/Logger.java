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

import java.io.PrintStream;

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
	private static final boolean debugEnabled = false;
	
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
	public static void log(Object obj) {
		if (debugEnabled) {
			outStream.println(obj.toString());
		}
	}
	
	/**
	 * Writes the specified message to the output stream.
	 * @param msg The message that should be printed.
	 **/
	public static void log(String msg) {
		if (debugEnabled) {
			outStream.println(msg);
		}
	}
	
	/**
	 * Writes the specified message to the output stream using the PrintStream format method.
	 * @param msg A format string as described in http://docs.oracle.com/javase/7/docs/api/java/util/Formatter.html
	 * @param objects Arguments referenced by the format specifiers in the format string.
	 **/
	public static void log(String msg, Object... objects) {
		if (debugEnabled) {
			outStream.format(msg, objects);
		}
	}
	
	/**
	 * Prints the stacktrace for the specified throwable to the error stream.
	 * @param throwable The throwable that should be printed.
	 **/
	public static void log(Throwable throwable) {
		if (debugEnabled) {
			throwable.printStackTrace(errStream);
		}
	}
}
