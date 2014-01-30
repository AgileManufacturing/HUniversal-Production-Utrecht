/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file src/REXOS/MAS/simulation/CSVReader.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-17
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Roy Scheefhals
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright � 2013, HU University of Applied Sciences Utrecht. 
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package simulation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import simulation.data.Capability;

public class CSVReader {
	
	//example : "1;name,1;name,3;appel,4;peer,5;hurp,2;durp"
	
	
	private static Capability[] availableCapabilities;

	public static String[][] parseCsvFile(String csvFilePath) {
		System.out.println("reading " + csvFilePath);
		
		String fileString = "";
		try {
			byte[] fileContent = Files.readAllBytes(Paths.get(csvFilePath));
			fileString = Charset.defaultCharset().decode(ByteBuffer.wrap(fileContent)).toString();
		} catch (FileNotFoundException ex) {
			System.err.println("Could not find file: " + csvFilePath);
			ex.printStackTrace();
		} catch (IOException ex) {
			System.err.println("Error when reading from file: " + csvFilePath);
			ex.printStackTrace();
		}
		return parseCsvString(fileString);
	}
	public static String[][] parseCsvString(String csvString) {
		// split on lines (this should work for windows too)
		String lines[] = csvString.split("\\r?\\n");
		int rowCount = lines.length;
		if(rowCount == 0) {
			// if input is empty, so is output
			return new String[0][0];
		}
		
		String[][] output = new String[rowCount][];
		for(int i = 0; i < lines.length; i++) {
			int colCount = lines[i].split(",").length;
			output[i] = new String[colCount];
			String[] fields = lines[i].split(",");
			output[i] = fields;
		}
		return output;
	}
	
	public static JsonObject parseJsonObjectFile(String JsonFilePath){
		
		String fileString = "";
		try {
			byte[] fileContent = Files.readAllBytes(Paths.get(JsonFilePath));
			fileString = Charset.defaultCharset().decode(ByteBuffer.wrap(fileContent)).toString();
		} catch (FileNotFoundException ex) {
			System.err.println("Could not find file: " + JsonFilePath);
			ex.printStackTrace();
		} catch (IOException ex) {
			System.err.println("Error when reading from file: " + JsonFilePath);
			ex.printStackTrace();
		}
		if (fileString.equals("")){
			System.err.println("File: " + JsonFilePath + ", is empty!");
		}
		return parseJsonObjectString(fileString);
	}
	
	public static JsonObject parseJsonObjectString(String jsonString){
		JsonParser parser = new JsonParser();
		JsonObject obj = (JsonObject) parser
				.parse(jsonString).getAsJsonObject();
		return obj;
	}
}
