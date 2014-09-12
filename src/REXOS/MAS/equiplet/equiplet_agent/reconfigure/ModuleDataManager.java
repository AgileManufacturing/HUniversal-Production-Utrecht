/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ModuleDataManager
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class helps retrieve data from the USB and write data to it.
 *          dM@      dMMM3  .ga...g,    	@date Created:	201-04-03
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Alexander Hustinx
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
package MAS.equiplet.equiplet_agent.reconfigure;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;


public class ModuleDataManager {

	/**
	 * @var TRANSFER_USB_NAME
	 * 	The name of the USB device used for data transfer
	 *  Make sure to end this string with "/"
	 */
	private final String TRANSFER_USB_NAME = "USB/";

	/**
	 * @var USB_PATH
	 *  The path where the USB device can be found
	 *  This is excluding the name!
	 *  
	 *  In Ubuntu 12.04 LTS this is "/media/" by default
	 */
    private final String USB_PATH = "/media/"; 

    /**
     * @var modulePath
     *  The string containing the path the Module should have
     *  based on "_MOD_/manufacturerID/typeNumber/serialNumber"
     */
	private String modulePath;
	
	/**
	 * @var modulePathOnUSB
	 *  The string containing the path the Module should have to and on the USB
	 *  This means USB_PATH + TRANSFER_USB_NAME + modulePath;
	 */
	private String modulePathOnUSB;

	@SuppressWarnings("unused")
	private ModuleDataManager(){
		//SHOULD NEVER BE CALLED
	}

	/**
	 * Constructor for the ModuleDataManager
	 * 
	 * @param qrString		The string of the scanned QR code on the Module
	 * 
	 * @throws Exception	Exception that might occur when the inputed QR code string is either not a "_MOD_"-string 
	 * 						or does not fit the template found on http://wiki.agilemanufacturing.nl/index.php/Reconfiguration
	 */
	public ModuleDataManager(String qrString) throws Exception{
		modulePathOnUSB = convertQrStringToPath(qrString);
	}

	/**
	 * function used to translate the given QR code string into a valid path on the USB
	 * 
	 * @param qrString		The string of the scanned QR code on the Module
	 * 
	 * @return				The Module's path on the USB
	 * 
	 * @throws Exception	Exception that might occur when the inputed QR code string is either not a "_MOD_"-string 
	 * 						or does not fit the template found on http://wiki.agilemanufacturing.nl/index.php/Reconfiguration
	 */
	private String convertQrStringToPath(String qrString) throws Exception{
		/** 
		 * The template for the QR codes is:	"_MOD_|manufacturerID|typeNumber|serialNumber"
		 * 	When this template is not met, exception can occur, and the output will not be correct!
		 */
		String[] splittedQrString = qrString.split("\\|");

		if(!splittedQrString[0].equals("_MOD_")){
			throw new Exception("QR \"" + qrString + "\" is not a Module QR code!");
		}

		else if(splittedQrString.length != 4){
			throw new Exception("QR \""+ qrString + "\" has the wrong format! \nRequired format: \"_MOD_|<manufacturerID>|<moduleTypeID>|<serialID>\"");
		}

		else{
			modulePath = splittedQrString[0] + "/"
					+ splittedQrString[1] + "/" 
					+ splittedQrString[2] + "/" 
					+ splittedQrString[3] + "/";

			modulePathOnUSB = USB_PATH + TRANSFER_USB_NAME + modulePath;
			return modulePathOnUSB;
		}
	}

	/**
	 * function that converts the first file, in the give folder, with the given extension, to a byte array
	 * 
	 * @param folderPath		the path in which you want to find a file to convert
	 * 
	 * @param requiredExtention	the file-extension you are searching
	 * 
	 * @return					the byte array of the first file with the given extension, found in the given folder
	 * 
	 * @throws Exception		FileNotFoundException will occur if the given folder does not exist
	 * 							Exception will occur if there was no file with the given extension found in the given folder
	 */
	private byte[] fileToByteArray(String folderPath, String requiredExtention) throws Exception {

		File file = new File(folderPath);
		String filename = "";

		if(!file.exists()) {
			throw new FileNotFoundException("Folder \""+ file.getAbsolutePath() + "\" was not found!");
		} 

		boolean isExtentionFound = false;
		String[] folderContent = file.list();
		for(int i = 0; i < folderContent.length; i++) {
			/** check each file in the given folder for the given extension */
			String fileExtention;
			try {
				fileExtention = folderContent[i].split("\\.")[1];	
			} catch(IndexOutOfBoundsException e) {
				/** should never be able to occur ... */
				fileExtention = "";
			}
			
			if(fileExtention.equals(requiredExtention)) {
				isExtentionFound = true;
				filename = folderContent[i];
			}
		}

		if(!isExtentionFound) {
			throw new Exception("Folder \"" + file.getAbsolutePath() + "\" did not contain a \"" + requiredExtention + "\"-file!");
		} else {
			try {
				/** 
				 * File with the desired extension was found.
				 * The desired file will be converted to a byte array and returned
				 */
				File desiredFile = new File(folderPath + filename);
				return Files.readAllBytes(desiredFile.toPath());
			} catch (IOException e) {
				throw new IOException(e);
			}
		}
	}

	@Deprecated
	/**
	 * function to get the byte array of the last file with a "*.jar"-extension within the modulePathOnUSB
	 * 
	 * @return				byte array of the last file with a "*.jar"-extension within the modulePathOnUSB
	 * 
	 * @throws Exception	FileNotFoundException will occur if the given folder does not exist
	 * 						Exception will occur if there was no file with the given extension found in the given folder
	 */
	public byte[] getJarFileAsByteArray() throws Exception{
		try {
			return fileToByteArray(modulePathOnUSB + "ModuleData/", "jar");
		} catch (Exception e){
			throw e;
		}
	}

	@Deprecated
	/**
	 * function to get the byte array of the last file with a "*.zip"-extension within the modulePathOnUSB
	 * 
	 * @return				byte array of the last file with a "*.zip"-extension within the modulePathOnUSB
	 * 
	 * @throws Exception	FileNotFoundException will occur if the given folder does not exist
	 * 						Exception will occur if there was no file with the given extension found in the given folder
	 */
	public byte[] getZipFileAsByteArray() throws Exception{
		try {
			return fileToByteArray(modulePathOnUSB + "ModuleData/", "zip");
		} catch (Exception e){
			throw e;
		}
	}

	/**
	 * function to get the string of the last "*.json"-file within the modulePathOnUSB
	 * 
	 * @return				string with the contents of the last "*.json"-file within the modulePathOnUSB (in UTF-8)
	 * 
	 * @throws Exception	FileNotFoundException will occur if the given folder does not exist
	 * 						Exception will occur if there was no file with the given extension found in the given folder
	 */
	public String getJsonFileAsString() throws Exception{

		try {
			byte[] encoded = fileToByteArray(modulePathOnUSB + "ModuleData/", "json");
			return StandardCharsets.UTF_8.decode(ByteBuffer.wrap(encoded)).toString();
		} catch (Exception e){
			throw e;
		}
	}

	/**
	 * function to write a json-string into the "staticData.json"-file within the modulePathOnUSB
	 * 
	 * @param jsonString	the string that will be written into the file
	 * 
	 * @return				returns true when successful; and false when the string could not be 
	 * 						written into a file
	 */
	public boolean writeStringToJsonFile(String jsonString){

		File jsonFolder = new File(modulePathOnUSB + "ModuleData/");
		File jsonFile = null;

		/** 
		 * checks if the folder and if the staticData.json exists,
		 * create the folder if it does not exists,
		 * deletes the file if it exists.
		 */
		if(!jsonFolder.exists()){
			/** 
			 * if the folder does not exist, attempt to create it,
			 * when it can not be created return false,
			 * if it is successfully created call this function again.
			 */
			if(!jsonFolder.mkdirs()){
				return false;
			} else {
				return writeStringToJsonFile(jsonString);
			}
		} else {
			/** delete the "staticData.json" if it exists */
			jsonFile = new File(jsonFolder.getAbsolutePath() + "/staticData.json");
			if(jsonFile.exists()){
				System.out.println("Deleting the existing .json-file");
				jsonFile.delete();
			}
		}

		/** write the given json-string to the "staticData.json"-file */
		BufferedWriter bw = null;
		try {
			bw = new BufferedWriter(new FileWriter(jsonFile));
			bw.write(jsonString);
			bw.flush();
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		} finally {
			try {
				bw.close();
			} catch (IOException e) {
				e.printStackTrace();
				System.out.println("Well ... we tried ...");
			}
		}

		return true;
	}

}