/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	CreateAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief  The CreateAgent creates a productagent spawner on the server it's JADE platform.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-06-06
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Bas Voskuijlen
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
package web_socket_server;


import java.net.URI;
import java.net.URISyntaxException;

import configuration.ServerConfigurations;

import web_socket_server.java.org.java_websocket.client.WebSocketClient;
import web_socket_server.java.org.java_websocket.handshake.ServerHandshake;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentController;
import jade.wrapper.ControllerException;


public class CreateAgent {	
	/**
	  * @var CONTAINER_NAME
	  * The string holds the container name where in the EquipletAgent is being spawned.
	  */
	private static final String CONTAINER_NAME = "ProductAgentSpawnerAgent";

	
	 /**
	  * CreateAgent()
	  * Spawns the ProductAgent on the server.
	  * @throws ControllerException 
	  */
	public void createAgent(String args, int identifier){
		java.util.Date date= new java.util.Date();
		//Spawning EquipletAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		System.out.println("Creating agent");
		
		try {
			Profile profile = new ProfileImpl();
			
			profile.setParameter(Profile.MAIN_HOST,ServerConfigurations.GS_IP);
			profile.setParameter(Profile.MAIN_PORT,ServerConfigurations.GS_PORT);
			profile.setParameter(Profile.CONTAINER_NAME,CONTAINER_NAME+date.getTime());
			jade.wrapper.AgentContainer container = runtime.createAgentContainer( profile );
			ProductAgentSpawnerAgent agent = new ProductAgentSpawnerAgent();
			agent.setProductSteps(args);
			MyWebsocket mws = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
			try{		
				AgentController ac = container.acceptNewAgent( container.getContainerName()+identifier, agent);
				
				ac.start();	
				
				System.out.println("Starting AgentController: "+ ac.getState());
				mws.setCreated(true);
			}
			catch(ControllerException e){
				mws.setCreated(false);
			}
			mws.connect();
		}
		catch(Exception ex){
			try {
				MyWebsocket mws = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
				mws.setCreated(false);
				mws.connectBlocking();
			} catch (URISyntaxException e) {
				e.printStackTrace();
				System.out.println("Impossible, URL format incorrect: " + ServerConfigurations.WSS_URI);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	private class MyWebsocket extends WebSocketClient {
		boolean created = false;
		
		public void setCreated(boolean created){ this.created = created; }
		
		public MyWebsocket(URI serverURI) { super(serverURI); }

		@Override
		public String getResourceDescriptor() { return null; }

		@Override
		public void onOpen(ServerHandshake handshakedata) {
			// TODO Auto-generated method stub
			System.out.println("onOpen");
			if (created){
				send("{\"receiver\":\"interface\", \"message\":\"Created product\", \"type\":\"success\"}");
			}
			else {
				send("{\"receiver\":\"interface\", \"message\":\"Could not create product, connection error!!\", \"type\":\"danger\"}");
			}
			
			close();
		}

		@Override
		public void onMessage(String message) { }

		@Override
		public void onClose(int code, String reason, boolean remote) { }

		@Override
		public void onError(Exception ex) { }		
	}
}
