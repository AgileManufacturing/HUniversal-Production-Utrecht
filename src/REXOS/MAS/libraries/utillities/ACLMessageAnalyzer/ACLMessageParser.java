/**
 * @file rexos/libraries/log/Logger.java
 * @brief Processes logged messages to get testresults
 * @date Created: 17 september 2013
 *
 * @author Roy Scheefhals
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
package libraries.utillities.ACLMessageAnalyzer;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import agents.data.PrintableACLMessage;

import com.sun.xml.internal.bind.v2.runtime.unmarshaller.XsiNilLoader.Array;

public class ACLMessageParser {
	
	private ArrayList<File> ACLMsgsFiles;
	private ArrayList<ArrayList<PrintableACLMessage>> SavedACLMessages = new ArrayList<>();
	
	public ACLMessageParser(String messagesDirPath,  String resultPath){
		File messagesDir = new File(messagesDirPath);
		if (!messagesDir.exists()){
			throw new IllegalArgumentException("messages directory path has to be an existing folder");
		}
		if (!messagesDir.isDirectory()){
			throw new IllegalArgumentException("messages directory path has to be a folder, not a file");
		}
		
		ACLMsgsFiles = new ArrayList<File>();
		
		File[] possibleACLMsgsFiles = messagesDir.listFiles();
		
		//only get the proper files from the folder
		for (File posACLMsgsFile : possibleACLMsgsFiles){
			
			if (posACLMsgsFile.exists() && posACLMsgsFile.isFile() && 
					posACLMsgsFile.getName().endsWith(".log") && 
					!posACLMsgsFile.getName().startsWith("null")){
				ACLMsgsFiles.add(posACLMsgsFile);
			}
		}
		if (ACLMsgsFiles.size() < 1){
			throw new NullPointerException("messages directory does not contain any files");
		}
		
		//parse all the files and get its data ( all in memory )
		for (File curACLMsgsFile : ACLMsgsFiles){
			if (!curACLMsgsFile.getName().startsWith("null")){
				System.out.println("adding file: " + curACLMsgsFile.getName());
				SavedACLMessages.add(loadSavedACLMessages(curACLMsgsFile));
			}
		}
	}
	
	public ACLMessageTestResult analyzeData(){
		ACLMessageTestResult results = new ACLMessageTestResult();
		results.amountProductSteps = SavedACLMessages.size();
		//process every list
		for (ArrayList<PrintableACLMessage> curList : SavedACLMessages){
			for( int iMessage = 0 ; iMessage < curList.size(); iMessage ++){
				PrintableACLMessage curMsg = curList.get(iMessage);
				long timedifference = 0;
				//match message for its counterpart
				boolean counterpartFound = false;
				if (iMessage + 1 < curList.size()){
				PrintableACLMessage expectedCounterpartMsg = curList.get(iMessage + 1);
				
					if (expectedCounterpartMsg.messageID.equals(curMsg.messageID)){
						// we have our counterpart, continue processing
						timedifference = expectedCounterpartMsg.timestamp - curMsg.timestamp;
						counterpartFound = true;
						curList.remove(iMessage + 1);
					}
				}
				//did not found counterpart at the expected place, check all the other messages
				if ( !counterpartFound){
					for ( int iMsgCheck = 0 ; iMsgCheck < curList.size(); iMsgCheck++){
						PrintableACLMessage curCounterpartMsg = curList.get(iMsgCheck);
						if (curCounterpartMsg.messageID.equals(curMsg.messageID)){
							timedifference = curCounterpartMsg.timestamp - curMsg.timestamp;
							curList.remove(iMsgCheck);
							counterpartFound = true;
							break;
						}
					}
					// we did not found the counterpart, count up the lost messages and continue
					if (!counterpartFound){
						results.amountLostMessages++;
						continue;
					}
				}

				//we got our message and its counterpart exists
				//now parse it in the results
				results.amountMessages++;
				results.addSendTime(timedifference);
				results.addContentSize(curMsg.contentSize);
				
			}
		}

		System.out.println(results.getMessagesInfo());
		System.out.println(results.getContentSizeInfo());
		System.out.println(results.getSendTimeInfo());
		return results;
	}
	
	private ArrayList<PrintableACLMessage> loadSavedACLMessages(File ACLMessageFile){
		ArrayList<PrintableACLMessage> parsedMessages = new ArrayList<PrintableACLMessage>();
		
			try {
				BufferedReader br = new BufferedReader(new FileReader(ACLMessageFile));
				String curLine = "";
				while ((curLine = br.readLine()) != null){
					PrintableACLMessage curmsgs= PrintableACLMessage.createACLMessageFromSavedLine(curLine);
					parsedMessages.add(curmsgs );
				}
				br.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		System.out.println("added " + parsedMessages.size() + " msgs");
		return parsedMessages;
	}
}