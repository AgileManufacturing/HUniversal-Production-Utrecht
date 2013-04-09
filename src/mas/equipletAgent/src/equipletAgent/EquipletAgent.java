/**
 * @file EquipletAgent.java
 * @brief Provides an equiplet agent that communicates with product agents and with its own service agent.
 * @date Created: 2013-04-02
 *
 * @author Hessel Meulenbeld
 * @author Thierry Gerritse
 * @author Wouter Veen
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
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

package equipletAgent;

import behaviours.CanDoProductionStepResponse;
import behaviours.CanPerformStep;
import behaviours.GetProductionDuration;
import behaviours.ProductionDurationResponse;
import behaviours.ScheduleStep;

import com.mongodb.*;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.InstanceCreator;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.ReceiverBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.MessageTemplate.MatchExpression;
import jade.lang.acl.UnreadableException;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import newDataClasses.ParameterList;
import newDataClasses.ProductionStep;
import nl.hu.client.BasicOperationSubscription;
import nl.hu.client.BlackboardClient;
import nl.hu.client.BlackboardSubscriber;
import nl.hu.client.BlackboardSubscription;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;
import nl.hu.client.MongoOperation;
import nl.hu.client.OplogEntry;
import serviceAgent.ServiceAgent;
import behaviours.CanPerformStep;

import java.io.IOException;
import java.io.Serializable;
import java.lang.reflect.Type;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Hashtable;

import org.bson.BSONObject;
import org.bson.types.ObjectId;

/**
 * EquipletAgent that communicates with product agents and with its own service
 * agent.
 **/
public class EquipletAgent extends Agent implements BlackboardSubscriber{
	private static final long serialVersionUID = 1L;

	// This is the service agent of this equiplet
	private AID serviceAgent;
	
	// This is the collective database used by all product agents and equiplets
	// and contains the collection EquipletDirectory.
	private DB collectiveDb = null;
	private String collectiveDbIp = "145.89.191.131";
	private int collectiveDbPort = 27017;
	private String collectiveDbName = "CollectiveDb";
	private String equipletDirectoryName = "EquipletDirectory";

	// This is the database specific for this equiplet, this database contains
	// all the collections of the equiplet, service and hardware agents.
	private DB equipletDb = null;
	private String equipletDbIp = "localhost";
	private int equipletDbPort = 27017;
	private String equipletDbName = "";
	private DBCollection productSteps = null;
	private String productStepsName = "ProductStepsBlackBoard";

	//BlackboardClient to communicate with the blackboards
	private BlackboardClient collectiveBBclient, equipletBBclient;

	//Arraylist with IDs of the capabilities of the equiplet
	private ArrayList<Long> capabilities;

	private Hashtable<String, ObjectId> communicationTable;

	@SuppressWarnings("unchecked")
	public void setup() {
		System.out.println("I spawned as a equiplet agent.");
		
		// set the database name to the name of the equiplet and set the database ip to its own IP.
		try{
			InetAddress IP = InetAddress.getLocalHost();
			equipletDbIp = IP.getHostAddress();
		}catch(Exception e){
			e.printStackTrace();
		}
		equipletDbName = getAID().getLocalName();
		communicationTable = new Hashtable<String, ObjectId>();
		// TODO: Not Hardcoded capabilities/get capabilities from the service agent.
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			capabilities = (ArrayList<Long>) args[0];
			System.out.format("%s %s%n", capabilities, equipletDbName);
		}
        try {
        	DbData dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);
        	Object[] arguments = new Object[]{dbData};
			((AgentController)getContainerController().createNewAgent(getLocalName() + "-serviceAgent", "serviceAgent.ServiceAgent", arguments)).start();
			serviceAgent = new AID((String) getLocalName() + "-serviceAgent", AID.ISLOCALNAME);
		} catch (StaleProxyException e1) {
			e1.printStackTrace();
			doDelete();
		}
        
        try {
        	collectiveBBclient = new BlackboardClient(collectiveDbIp);
			collectiveBBclient.setDatabase(collectiveDbName);
	        collectiveBBclient.setCollection(equipletDirectoryName);
	        
	        equipletBBclient = new BlackboardClient(equipletDbIp);
	        equipletBBclient.setDatabase(equipletDbName);
	        equipletBBclient.setCollection(productStepsName);
	        equipletBBclient.subscribe(new BasicOperationSubscription(MongoOperation.UPDATE, this));
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException e) {
			e.printStackTrace();
			doDelete();
		}
		
		Gson gson = new Gson();
		// put capabilities on the equipletDirectory
		try {
			DbData dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);
			EquipletDirectoryMessage entry = new EquipletDirectoryMessage(getAID(), capabilities, dbData);
			collectiveBBclient.insertDocument(gson.toJson(entry));
		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}

		CanPerformStep canPerformStepBehaviour = new CanPerformStep(this);
        addBehaviour(canPerformStepBehaviour);
		
		CanDoProductionStepResponse canDoProductionStepResponseBehaviour = new CanDoProductionStepResponse(this);
		addBehaviour(canDoProductionStepResponseBehaviour);
		
		GetProductionDuration getProductionDurationBehaviour = new GetProductionDuration(this);
		addBehaviour(getProductionDurationBehaviour);
	
		ProductionDurationResponse productionDurationResponseBehaviour = new ProductionDurationResponse(this);
		addBehaviour(productionDurationResponseBehaviour);
		
		ScheduleStep scheduleStepBehaviour = new ScheduleStep(this);
		addBehaviour(scheduleStepBehaviour);
	}
	
	public void takeDown() {
		Gson gson = new Gson();
		try {
			BasicDBObject searchQuery = new BasicDBObject();
			searchQuery.put("AID", getAID());
			collectiveBBclient.removeDocuments(gson.toJson(searchQuery));
		} catch (Exception e) {
			e.printStackTrace();
			// The equiplet is already going down, so it has to do nothing here.
		}
		// TODO: message to PA's
		// TODO: remove own database
	}
	
	public BlackboardClient getEquipletBBclient(){
		return equipletBBclient;
	}
	
	public void addCommunicationSlot(String conversationId, ObjectId objectId){
		communicationTable.put(conversationId, objectId);
	}
	
	public ObjectId getCommunicationSlot(String conversationId){
		return communicationTable.get(conversationId);
	}
	
	public AID getServiceAgent(){
		return serviceAgent;
	}
	
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		// TODO Implement onMessage
	}
}
