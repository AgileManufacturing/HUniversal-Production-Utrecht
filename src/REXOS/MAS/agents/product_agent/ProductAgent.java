/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ProductAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This agent wants his product steps to be completed. Communication with the EquipletAgents is mandatory.
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
package agents.product_agent;

import generic.ProductStep;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;

import agents.data_classes.MessageType;
import agents.data_classes.Proposal;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;

/**
 * ProductAgent that communicates with equipletagents to plan its product steps.
 **/
public class ProductAgent extends Agent{
	private static final long serialVersionUID = 1L;
	/**
	  * @var SUCCESSFULL_PLANNED_PRODUCTSTEP
	  * The message received by the equiplet agent when a product is succesfully planned.
	  */
	private static final String SUCCESSFULL_PLANNED_PRODUCTSTEP = "true";
	/**
	  * @var PRODUCT_STEPS
	  * The key in the JSON received when the agents constructs.
	  */
	private static final String PRODUCT_STEPS = "productsteps";
	
	/**
	  * @var productStepList
	  * Contains all product steps received by 0th argument.
	  */
	private JsonArray productStepList;
	/**
	  * @var currentPlannedProductStep
	  * The product step to plan into equiplet agents.
	  */
	private int currentPlannedProductStep = 0;

	protected void setup(){
		Object[] arguments = this.getArguments();
		if (arguments.length <= 0){
			System.out.println("No arguments received! Expected product steps in json format");
		}
		else {
			try {
				JsonElement productSteps = new JsonParser().parse(arguments[0].toString());
				if (productSteps != null){
					productStepList = productSteps.getAsJsonObject().getAsJsonArray(PRODUCT_STEPS);
					System.out.println("Productagent received product steps: " + productSteps.toString());
					planCurrentProductStep();
				}
			} catch (JsonSyntaxException ex) {
				System.out.println("Invalid JSON format! " + ex);
			}
		}
		
		
		
		
		
		
		addBehaviour(new CyclicBehaviour() {
			private static final long serialVersionUID = 1L;
			private ArrayList<Proposal> proposals = new ArrayList<Proposal>();

			public void action() {
				ACLMessage msg = receive();
				if (msg!=null) {
					System.out.println(msg.getSender().getName()+" Send: "+msg.getContent() );

					if (msg.getPerformative() == MessageType.AVAILABLE_TO_PLAN){
						JsonObject proposal = new JsonParser().parse(msg.getContent()).getAsJsonObject();
						proposals.add(new Proposal(proposal,msg.getSender()));
						if (proposals.size() == 1){
							for (int i=0;i<productStepList.size();i++){
								String pid = productStepList.get(i).getAsJsonObject().get("productStepId").getAsString();
								if (pid.equals(proposal.get("productStepId").getAsString())){
									proposal.add("productStep",productStepList.get(i));
									i = productStepList.size();
								}
							}
							System.out.println("Sending plan message: " + proposal.toString());
							
				            sendMessage(MessageType.PLAN_PRODUCT_STEP, 
				            			getAID(), 
				            			proposals.get(0).getEquipletAgent(), 
				            			proposal.toString(),
				            			"meta");
						}
					} 
					else if (msg.getPerformative() == MessageType.CONFIRM_PLANNED) {
						//If the product step was successfully planned, go to next product step.
						if (msg.getContent().equals(SUCCESSFULL_PLANNED_PRODUCTSTEP)){
							currentPlannedProductStep++;
						}
						proposals.clear();
						planCurrentProductStep();
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
	
	private void planCurrentProductStep(){
		if (productStepList.size() > currentPlannedProductStep){
			ProductStep productStep = (new ProductStep(productStepList.get(currentPlannedProductStep).getAsJsonObject()));

			
			DFAgentDescription dfd = new DFAgentDescription();
			ServiceDescription sd = new ServiceDescription();
			sd.setName(productStep.getService().getName());
			sd.setType(productStep.getService().getName());
			dfd.addServices(sd);
			try {
				DFAgentDescription[] equipletAgents;
				equipletAgents = DFService.search(this, dfd);
				JsonObject message = productStepList.get(0).getAsJsonObject();
				message.addProperty("startTime", currentPlannedProductStep);
				for (int j=0;j<equipletAgents.length;j++){
					AID aid = equipletAgents[j].getName();
					sendMessage(MessageType.CAN_EXECUTE_PRODUCT_STEP, getAID(), aid, message.getAsString(), "meta");
				}
				if (equipletAgents.length == 0){
					System.out.println("No equiplets found to execute this service: " + productStep.getService().getName());
				}
			} catch (FIPAException e) {
				System.out.println("DF Search Error");
				e.printStackTrace();
			}
			
		}
		else { //Finished planning product steps
			System.out.println("No more product steps to plan");
			System.out.println("Product Agent AWAYYYYYYY!!!!");
			this.doDelete();
		}
	}
	
	/**
	  * sendMessage()
	  * The function sendMessage sends an ACL message to only one other agent.
	  * The language is mostly "meta".
	  */
	private void sendMessage(int messageType, AID sender, AID receiver, String content, String language){
		ACLMessage message = new ACLMessage( messageType );
		message.setSender(sender);
		message.addReceiver(receiver);
		message.setLanguage(language);
		message.setContent(content);
		send(message);
	}
}
