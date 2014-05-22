/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	EQSpawnClass
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class is responsible for creating the EquipletAgent.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-05-20
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
import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.wrapper.AgentController;

import agents.equiplet_agent.EquipletAgent;

public class EQSpawnClass extends Agent {
	/**
	  * @var MAIN_HOST
	  * The string has the IP of the server that is hosting the Grid Agent Container.
	  */
	private static final String MAIN_HOST = "145.89.126.159";
	
	/**
	  * @var MAIN_PORT
	  * The string has the PORT of the server that is hosting the Grid Agent Container.
	  */
	private static final String MAIN_PORT = "1234";
	
	/**
	  * @var CONTAINER_NAME
	  * The string holds the container name where in the EquipletAgent is being spawned.
	  */
	private static final String CONTAINER_NAME = "EquipletAgent";

	
	 /**
	  * main()
	  * Spawns the EquipletAgent on the selected server.
	  */
	public static void main(String[] args) throws Exception {
		 java.util.Date date= new java.util.Date();
		//Spanwing EquipletAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		Profile profile = new ProfileImpl();		
		profile.setParameter(profile.MAIN_HOST,MAIN_HOST);
		profile.setParameter(profile.MAIN_PORT,MAIN_PORT);
		profile.setParameter(profile.CONTAINER_NAME,CONTAINER_NAME);
		
		jade.wrapper.AgentContainer container = runtime.createAgentContainer( profile );
		Agent agent = new EquipletAgent();
		agent.setArguments(args);
		AgentController ac = container.acceptNewAgent( "EquipletAgent"+date.getTime(), agent);
		ac.start();
		
	}
}