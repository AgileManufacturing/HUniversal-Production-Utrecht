/**
 * @file rexos/libraries/log/Logger.java
 * @brief Data class for a saved ACLMessage
 * @date Created: 18 september 2013
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
package agents.data;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.sql.Timestamp;
import java.util.StringTokenizer;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

public class PrintableACLMessage {

	public String messageID = "";
	public long timestamp;
	public String sentOrReceived = "";
	public String sender = "";
	public String receiver = "";
	public int performative = -1;
	public String ontology = "";
	public int contentSize = 0;

	public PrintableACLMessage(ACLMessage msg, Timestamp timestamp, String sentOrReceived){
		this.messageID = msg.getUserDefinedParameter("message-id");
		this.timestamp = timestamp.getTime();
		this.sentOrReceived = sentOrReceived;
		this.sender = msg.getSender().getLocalName();
		this.receiver = ((AID)msg.getAllReceiver().next()).getLocalName();
		this.performative = msg.getPerformative();
		this.ontology = msg.getOntology();
		if(msg.getByteSequenceContent() != null) {
			contentSize = msg.getByteSequenceContent().length;
		}
	}
	
	public PrintableACLMessage(){
		
	}
	
	/**
	 * creates a new SavedACLMessage from a saved line defined in the Logger
	 * 
	 * @param fromSavedLine
	 */
	public static PrintableACLMessage createACLMessageFromSavedLine(String fromSavedLine){
		PrintableACLMessage aclMessage = new PrintableACLMessage();
		
		StringTokenizer st = new StringTokenizer(fromSavedLine, "~");
		aclMessage.messageID = st.nextToken();
		try{
			aclMessage.timestamp =	Long.parseLong(st.nextToken());
		}catch(NumberFormatException e){
			Logger.log(LogLevel.WARNING, "Could not parse the timestamp, returning PrintableACLMessage=null at msgID: " + aclMessage.messageID);
			return null;
		}
		aclMessage.sentOrReceived = st.nextToken();
		aclMessage.sender = st.nextToken();
		aclMessage.receiver = st.nextToken();
		try{
			aclMessage.performative = Integer.parseInt(st.nextToken());
		}catch(NumberFormatException e){
			Logger.log(LogLevel.WARNING, "Could not parse the performative, returning PrintableACLMessage=null at msgID: "
							+ aclMessage.messageID);
			return null;
		}
		aclMessage.ontology = st.nextToken();
		try{
			aclMessage.contentSize = Integer.parseInt(st.nextToken());
		}catch(NumberFormatException e){
			Logger.log(LogLevel.WARNING, "Could not parse the contentsize, contentsize stays 0 at msgID:"
						+ aclMessage.messageID);
		}
		
		return aclMessage;
	}
	
	@Override
	public String toString(){
		return String.format("%s~%d~%s~%s~%s~%d~%s~%d", messageID, timestamp, 
								sentOrReceived, sender, receiver, performative, 
								ontology, contentSize);
	}
}
