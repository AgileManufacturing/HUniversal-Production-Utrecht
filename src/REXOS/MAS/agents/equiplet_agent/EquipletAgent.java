/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	EquipletAgent
 *         .MN.      MMMMM;  ?^ ,THM		@brief  The EquipletAgent that is responsible for the communication between the Equiplet and the ProductAgent.
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
package agents.equiplet_agent;

import java.util.ArrayList;

import agents.data_classes.MessageType;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import HAL.HardwareAbstractionLayer;
import HAL.steps.HardwareStep;
import HAL.Module;
import HAL.steps.ProductStep;
import HAL.Service;
import HAL.listeners.HardwareAbstractionLayerListener;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import libraries.knowledgedb_client.KnowledgeException;

public class EquipletAgent extends Agent implements HardwareAbstractionLayerListener{
		
	private static final long serialVersionUID = -4551409467306407788L;
	
	/**
	  * @var equipletSchedule
	  * The ArrayList equipletSchedules holds all the Job objects that the EQ needs to execute.
	  */
	private ArrayList<Job> equipletSchedule= new ArrayList<Job>();
	
	/**
	  * @var serviceList
	  * The ArrayList serviceList holds all the services that the equiplet can execute.
	  */
	private ArrayList<Service> serviceList=null;
	
	/**
	  * @var scheduleCounter
	  * Counts the amount of schedules that have been executed.	  
	  */
	private int scheduleCounter = 0;
	
	/**
	  * @var hal
	  * The Object HAL is used to execute the planned jobs.
	  */
	private HardwareAbstractionLayer hal;
	
	
	/**
	  * setup()
	  * The function setup creates the HAL object and initialized the serviceList.
	  * The EquipletAgent then registers himself, with his service types to the DF.
	  * Main task is to wait for incoming ACLMessaged to communicate with ProductAgents.
	  */
	protected void setup(){				
		try {
			hal = new HardwareAbstractionLayer(this);
			System.out.println("HAL created");
			serviceList = hal.getSupportedServices();
		} catch (KnowledgeException e) {
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		DFAgentDescription dfd = new DFAgentDescription();
		for(int i =0; i < serviceList.size(); i++){
	        ServiceDescription sd  = new ServiceDescription();
	        sd.setName( serviceList.get(i).getName() );
	        sd.setType(serviceList.get(i).getName());
	        dfd.addServices(sd);
		}
		try {
			DFService.register(this, dfd );
		} catch (FIPAException e) {
			e.printStackTrace();
		}  

		addBehaviour(new CyclicBehaviour()
		{   
			private static final long serialVersionUID = 6925459044662459048L;

			public void action() {
				ACLMessage msg = receive();
				if (msg!=null) {
					System.out.println(msg.getSender() + msg.getContent());
					if(!msg.getSender().equals(this.getAgent().getAID())) {  
                    	if(msg.getPerformative()==MessageType.CAN_EXECUTE_PRODUCT_STEP){
                    		String result =canExecute(msg.getContent());
                    		System.out.println(result);
                    		if(!result.equals("false")){
                    			ACLMessage reply = msg.createReply();
        	                    reply.setPerformative( MessageType.AVAILABLE_TO_PLAN );
        	                    reply.setContent(result);
        	                    send(reply);
                    		}
                    	}                    	
                    	if(msg.getPerformative()==MessageType.PLAN_PRODUCT_STEP){
                    		String result =schedule(msg.getContent());
                			ACLMessage reply = msg.createReply();
    	                    reply.setPerformative( MessageType.CONFIRM_PLANNED );
    	                    reply.setContent(result);
    	                    send(reply);                    		
                    	}
					}
				}
				block();    
			}  
		});  
	}
	
	/**
	  * canExecute()
	  * This function checks if the requested service can be executed on this Equiplet.
	  * @return false if the service can't be executed, returns an startTime, duration and ID of the product step if it can be executed.
	  */
	private String canExecute(String msg){
		System.out.println("Can execute?");
		JsonObject productSteps = new JsonParser().parse(msg).getAsJsonObject();
		for(int i =0; i < serviceList.size(); i++){
	        if(serviceList.get(i).getName().equals(productSteps.get("service").getAsString())){
	        	
	        	JsonObject message = new JsonObject();
	        	message.addProperty("startTime", "0");
	        	message.addProperty("duration", "100");
	        	message.addProperty("productStepId", productSteps.get("id").getAsString());	       	
	        	
	        	return message.toString();
	        }	        
		}
		return "false";
	}
	/**
	  * schedule()
	  * This function schedules a job for the equiplet and places it in the equipletSchedule arraylist.
	  * @return true if the planning has been done succesfull.
	  */
	private String schedule(String msg){
		System.out.println("Can Schedule?");

		JsonObject productSteps = new JsonParser().parse(msg).getAsJsonObject();
		Job job = new Job(productSteps.get("startTime").getAsString(),productSteps.get("duration").getAsString(),productSteps.get("productStepId").getAsString(),productSteps.get("productStep").getAsJsonObject());
    	equipletSchedule.add(job);
		System.out.println(equipletSchedule.size()+" =Length");
		if(scheduleCounter == 0){
			executeProductStep();
		}
		return "true";		
	}
	
	/**
	  * takeDown()
	  * This function is being called on when the Agent is being terminated. 
	  * The EquipletAgent deregisters himself at the DF so that ProductAgents wont find him anymore.	  
	  */
	@Override
	protected void takeDown(){
		try { 
			DFService.deregister(this); 
			getContainerController().kill();				
		}
        catch (Exception e) {}
	}
	
	/**
	  * executeProductStep()
	  * This function lets the HAL executes a product step.
	  * The onTranslationFinished function is being called upon when this process has finished.
	  */
	private void executeProductStep(){
		if(scheduleCounter < equipletSchedule.size()){
			System.out.println("Execute PS");
			hal.translateProductStep(equipletSchedule.get(scheduleCounter).getProductStep());	
			scheduleCounter++;
		}else{
			scheduleCounter=0;
			equipletSchedule.clear();
		}
	}

	@Override
	public void onProcessStatusChanged(String state, long hardwareStepSerialId,
			Module module, HardwareStep hardwareStep) {
		if(state.equals("FAILED")){
			executeProductStep();
			//Log that process execute failed.
		}
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onTranslationFinished(ProductStep productStep,
			java.util.ArrayList<HardwareStep> hardwareStep) {
		System.out.println("Translating finished, Hardwarestep created");
		System.out.println(hardwareStep.size());
			hal.executeHardwareSteps(hardwareStep);
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
	}

	@Override
	public void onExecutionFinished() {
		System.out.println("Hardware Step Executed");
		executeProductStep();		
	}

	@Override
	public String getEquipletName() {
		// TODO Auto-generated method stub
		return "EQ1";
	}

	@Override
	public void onEquipletStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onEquipletModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}
	
}
