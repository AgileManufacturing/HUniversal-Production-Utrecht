/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	PartsAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This agent is responsible for finding the parts that a Product Agent requires.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-05-26
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Tom Oosterwijk
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2014, HU University of Applied Sciences Utrecht. 
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
package agents.parts_agent;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

import agents.data_classes.MessageType;


/**
 * PartsAgent that communicates with ProductAgent to locate the needed parts.
 **/
public class PartsAgent extends Agent{
	
	private static double[] lookUpTable = new double[32];
	
	private static final long serialVersionUID = 1L;
	private static final double maxStep = 17.0;
	private static final double smallStep = 5.5;
	private static String targetCrate = "GC4x4MB_6";
	double offsetX = -2.0;
	double offsetY = 1.6;
	
	private static String[] GC4x4MB_1 = new String[16];
	private static String[] GC4x4MB_4 = new String[16];
	private static String[] GC4x4MB_5 = new String[16];
	protected void setup(){	
		
		
		lookUpTable[0]=-17.0;
		lookUpTable[1]=17.0;
		
		lookUpTable[2]=-5.5;
		lookUpTable[3]=17.0;
		
		lookUpTable[4]=5.5;
		lookUpTable[5]=17.0;
		
		lookUpTable[6]=17.0;
		lookUpTable[7]=17.0;
		
		lookUpTable[8]=-17.0;
		lookUpTable[9]=5.5;
		
		lookUpTable[10]=-5.5;
		lookUpTable[11]=5.5;
		
		lookUpTable[12]=5.5;
		lookUpTable[13]=5.5;
		
		lookUpTable[14]=17.0;
		lookUpTable[15]=5.5;
		
		lookUpTable[16]=-17.0;
		lookUpTable[17]=-5.5;
		
		lookUpTable[18]=-5.5;
		lookUpTable[19]=-5.5;
		
		lookUpTable[20]=5.5;
		lookUpTable[21]=-5.5;
		
		lookUpTable[22]=17.0;
		lookUpTable[23]=-5.5;
		
		lookUpTable[24]=-17.0;
		lookUpTable[25]=-17.0;
		
		lookUpTable[26]=-5.5;
		lookUpTable[27]=-17.0;
		
		lookUpTable[28]=5.5;
		lookUpTable[29]=-17.0;
		
		lookUpTable[30]=17.0;
		lookUpTable[31]=-17.0;
		
		for(int i = 0; i < 16; i++){
			GC4x4MB_1[i]="blue";
		}
		
		for(int i = 0; i < 6; i++){
			GC4x4MB_4[i]="green";
		}
		
		for(int i = 0; i < 12; i++){
			GC4x4MB_5[i]="red";
		}
		addBehaviour(new CyclicBehaviour() {
			private static final long serialVersionUID = 1L;

			public void action() {
				ACLMessage msg = receive();
				if (msg!=null) {
					System.out.println(msg.getSender().getName()+" Send: "+msg.getContent() );

					if (msg.getPerformative() == MessageType.AVAILABLE_TO_PLAN){
						JsonObject partRequest = new JsonParser().parse(msg.getContent()).getAsJsonObject();
						partRequest.getAsJsonObject("criteria").getAsJsonObject("target").remove("identifier");
						partRequest.getAsJsonObject("criteria").getAsJsonObject("target").addProperty("identifier", targetCrate);
						JsonArray parts =findPart(partRequest.getAsJsonObject("criteria").getAsJsonObject("subjects").get("color").toString());
						
						partRequest.getAsJsonObject("criteria").remove("subjects");
						partRequest.getAsJsonObject("criteria").add("subjects", parts);
						ACLMessage reply = msg.createReply();
	                    reply.setPerformative( MessageType.SUPPLIER_REQUEST_REPLY );
	                    reply.setContent(partRequest.toString());
	                    send(reply);   
					}
					else {
						System.out.println(	"Received message is not any of capabile Performative MessageType! " + 
											"Could not process incomming message: " + msg.getContent());
					}
				}
				block();    
			}  
		});  
	}
	
	private JsonArray findPart(String color){
		JsonArray subjects = new JsonArray();
		JsonObject subject = new JsonObject();
		JsonObject subjectMove = new JsonObject();
		
		if(color.equals("red")){
			for(int i =0; i < GC4x4MB_5.length; i++){
				if(GC4x4MB_5[i].equals("red")){
					subjectMove.addProperty("x", (lookUpTable[(i*2)]+offsetX));
					subjectMove.addProperty("y", (lookUpTable[(i*2)+1]+offsetY));
					subjectMove.addProperty("z", "35");					
					subject.add("move",subjectMove);
					subject.addProperty("identifier", "GC4x4MB_5");
					subjects.add(subject);
					GC4x4MB_5[i]="";
				}
			}				

		} else if(color.equals("blue")){
			for(int i =0; i < GC4x4MB_1.length; i++){	
				if(GC4x4MB_1[i].equals("blue")){
					subjectMove.addProperty("identifier", "GC4x4MB_1");
					subjectMove.addProperty("x", (lookUpTable[(i*2)]+offsetX));
					subjectMove.addProperty("y", (lookUpTable[(i*2)+1]+offsetY));
					subjectMove.addProperty("z", "35");
					subject.add("move",subjectMove);
					subject.addProperty("identifier", "GC4x4MB_1");
					subjects.add(subject);
					GC4x4MB_1[i]="";
				}
			}
			
		} else if(color.equals("green")){
			for(int i =0; i < GC4x4MB_4.length; i++){	
				if(GC4x4MB_4[i].equals("green")){
					subjectMove.addProperty("identifier", "GC4x4MB_4");
					subjectMove.addProperty("x", (lookUpTable[(i*2)]+offsetX));
					subjectMove.addProperty("y", (lookUpTable[(i*2)+1]+offsetY));
					subjectMove.addProperty("z", "35");
					subject.add("move",subjectMove);
					subject.addProperty("identifier", "GC4x4MB_4");
					subjects.add(subject);
					GC4x4MB_4[i]="";
				}
			}
			
		}
		return subjects;		
	}
}
