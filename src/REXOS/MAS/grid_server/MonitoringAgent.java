/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	MonitoringAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This agent is responsible for forwarding all incoming msges that came from an equiplet to the Web Interface.
 *         									This is being done by user the WebSocketServer.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-06-06
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
package MAS.grid_server;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;

import java.net.URI;
import java.net.URISyntaxException;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import util.configuration.ServerConfigurations;

public class MonitoringAgent extends Agent {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private static final long MONITORING_INTERVAL = 10000;

	/**
	 * @var MyWebSocket The connection towards the websocket server. MonitoringAgent is able to send messages towards
	 *      the Webinterface using the SocketServer.
	 */
	MyWebsocket webSocket;

	protected void setup() {
		try {
			webSocket = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
			boolean connected = webSocket.connectBlocking();
		} catch (URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println(getLocalName() + " caught URISyntaxException: " + e.getMessage());
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		addBehaviour(new HeartBeatBehaviour(this, MONITORING_INTERVAL));
		addBehaviour(new MonitorListener());
		
		System.out.println(getLocalName() + " start with monitoring");
	}

	class MonitorListener extends CyclicBehaviour {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			ACLMessage msg = receive();
			if (msg != null) {
				// pass the information to the web socket
				webSocket.send(msg.getContent());
			}
			block();
		}
	}

	@Override
	protected void takeDown() {
		System.out.println(getLocalName() + ": terminated");
	}

	private class MyWebsocket extends WebSocketClient {
		public MyWebsocket(URI serverURI) {
			super(serverURI);
		}

		@Override
		public String getResourceDescriptor() {
			return null;
		}

		@Override
		public void onOpen(ServerHandshake handshakedata) {
			System.out.println(getLocalName() + ": websocket onOpen");
			// send("{\"receiver\":\"interface\", \"message\":\"Could not create product, connection error!!\", \"type\":\"danger\"}");
			// close();
		}

		@Override
		public void onMessage(String message) {
			// TODO Auto-generated method stub

		}

		@Override
		public void onClose(int code, String reason, boolean remote) {
			// TODO Auto-generated method stub
			System.out.println(getLocalName() + ": monitoring websocket onClose");

		}

		@Override
		public void onError(Exception ex) {
			// TODO Auto-generated method stub

		}
	}
}
