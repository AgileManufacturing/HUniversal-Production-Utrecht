/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	MessageType
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class creates new ACLMessageTypes that Agents can use.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-05-20
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
package grid_server;
import jade.lang.acl.ACLMessage;

public class MessageType {
	/**
	  * @var CAN_EXECUTE_PRODUCT_STEP
	  * The ACL message is used by the ProductAgent to ask an EquipletAgent if a product step can be executed on his Equiplet.
	  */
 public static final int CAN_EXECUTE_PRODUCT_STEP = ACLMessage.REQUEST; //Send By ProductAgent
 
 /**
  * @var PLAN_PRODUCT_STEP
  * The ACL message is used by the ProductAgent to inform an EquipletAgent that the ProductAgent wants to plan his product step.
  */
 public static final int PLAN_PRODUCT_STEP = ACLMessage.INFORM; // Send By ProductAgent
 
 /**
  * @var CONFIRM_PLANNED
  * The ACL message is used by the EquipletAgent to confirm the planning of a product step towards the ProductAgent.
  */
 public static final int CONFIRM_PLANNED = ACLMessage.CONFIRM; // Send By EquipletAgent
 
 /**
  * @var AVAILABLE_TO_PLAN
  * The ACL message is used by the EquipletAgent to inform the ProductAgent wether or not the requested product step can be executed by this equiplet.
  */
 public static final int AVAILABLE_TO_PLAN = ACLMessage.ACCEPT_PROPOSAL; // Send By EquipletAgent
 
 public static final int SUPPLIER_REQUEST = ACLMessage.PROPAGATE; // Send By EquipletAgent
 
 public static final int SUPPLIER_REQUEST_REPLY= ACLMessage.AGREE; // Send By EquipletAgent

 public static final int PULSE_UPDATE= ACLMessage.FAILURE; // Send By Monitoring Agent


}