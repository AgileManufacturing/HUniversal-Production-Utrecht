/**
 * @file tools/ACLMessageParser/ACLMessageTestResult
 * @brief Data object for the result of processing the logged ACLMessages
 * @date Created: 18 September 2013
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

public class ACLMessageTestResult {
	
	private int totalContentSize = 0;
	private long totalSendTime = 0;
	
	public int amountProductSteps = 0;
	public int amountMessages = 0;
	public int amountLostMessages= 0;
	public int amountMessagesContainingContent = 0;
	
	public double averageContentSize = 0;
	public int maxContentSize = 0;
	public int minContentSize = 0;
	
	public double averageSendTime = 0;
	public long minSendTime = 0;
	public long maxSendTime = 0;
	
	//public String biggestAgentSender = "";
	
	public ACLMessageTestResult(){
		
	}
	
	public void addSendTime(long time){
		totalSendTime += time;
		if (time > maxSendTime) maxSendTime = time;
		if (time < minSendTime) minSendTime = time;
		
		averageSendTime = (double)totalSendTime / (double)amountMessages;
	}
	
	public void addContentSize(int size){
		if ( size > 0){
			amountMessagesContainingContent ++;
			totalContentSize += size;
			if ( size > maxContentSize) maxContentSize = size;
			if ( size < minContentSize) minContentSize = size;
			
			averageContentSize = (double) totalContentSize / (double)amountMessagesContainingContent;
		}
	}
	
	public String getContentSizeInfo(){
		return String.format("Minimal ContentSize: %s, Maximum ContentSize: %s, Average contentSize: %s",
								minContentSize, maxContentSize, averageContentSize );
	}
	
	public String getSendTimeInfo(){
		return String.format("Minimal SendTime: %d, Maximum SendTime: %d, Average SendTime: %s",
				minSendTime, maxSendTime, averageSendTime);
	}
	
	public String getMessagesInfo(){
		return String.format("amount of production steps: %d\r\ntotal amount of messages: %d, "
				+ "messages with content: %d, lost messages: %d",
				amountProductSteps, amountMessages, amountMessagesContainingContent, amountLostMessages);
	}
}
