package rexos.mas.hardware_agent;

/**
 * @file rexos/mas/hardware_agent/HardwareAgent.java
 * @brief Provides an Hardware agent that communicates with Service agents and
 *        its own modules.
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;

import rexos.libraries.blackboard_client.BasicOperationSubscription;
import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.FieldUpdateSubscription;
import rexos.libraries.blackboard_client.FieldUpdateSubscription.MongoUpdateLogOperation;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;
import rexos.mas.data.DbData;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.hardware_agent.behaviours.CheckForModules;
import rexos.mas.hardware_agent.behaviours.EvaluateDuration;
import rexos.mas.hardware_agent.behaviours.FillPlaceholders;
import rexos.mas.hardware_agent.behaviours.ServiceAgentDied;
import rexos.mas.service_agent.ServiceStep;

import com.mongodb.BasicDBObject;

/**
 * HardwareAgent that communicates with the service agent and creates the messages for the Hardware layer.
 */
public class HardwareAgent extends Agent implements BlackboardSubscriber, ModuleUpdateListener {
	/**
	 * @var long serialVersionUID
	 * The serial version UID.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var BlackboardClient serviceStepBBClient
	 * The blackboard client for the serviceStep blackboard.
	 */
	private BlackboardClient serviceStepBBClient;

	/**
	 * @var BlackboardClient equipletStepBBClient
	 * The blackboard client for the equipletStep blackboard
	 */
	private BlackboardClient equipletStepBBClient;

	/**
	 * @var DbData dbData
	 * The DbData of this equiplet.
	 */
	private DbData dbData;

	/**
	 * @var HashMap<Integer, Integer> leadingModules
	 * A HashMap containing the leadingModules per step.
	 */
	private HashMap<Integer, Integer> leadingModules;

	/**
	 * @var ModuleFactory moduleFactory
	 * The moduleFactory of this agent.
	 */
	private ModuleFactory moduleFactory;

	/**
	 * @var AID equipletAgentAID
	 * The AID of the equipletAgent.
	 */
	private AID equipletAgentAID;

	/**
	 * @var AID serviceAgentAID
	 * The AID of the serviceAgent.
	 */
	private AID serviceAgentAID;

	/**
	 * @var HashMap<Integer, Object> configuration
	 * The configuration of this agent.
	 */
	private HashMap<Integer, Object> configuration;

	/**
	 * Function for registering a leading module.
	 * @param serviceId The service id to register for.
	 * @param moduleId The module id to register
	 */
	public void registerLeadingModule(int serviceId, int moduleId) {
		leadingModules.put(serviceId, moduleId);
	}

	/**
	 * Function for getting the leading module for a service id.
	 * @param serviceId The service id to get leading module for.
	 * @return the leading module id.
	 */
	public int getLeadingModule(int serviceId) {
		if (!leadingModules.containsKey(serviceId)) {
			return 0;
		}
		return leadingModules.get(serviceId);
	}

	/**
	 * @see Agent#setup()
	 */
	@Override
	public void setup() {
		Logger.log("Hardware agent " + this + " reporting.");
		leadingModules = new HashMap<Integer, Integer>();

		//gets the modulefactory and subscribes to updates.
		moduleFactory = new ModuleFactory();
		moduleFactory.subscribeToUpdates(this);

		//gets the dbData and AID from the arguments.
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
			equipletAgentAID = (AID) args[1];
			serviceAgentAID = (AID) args[2];
		}

		//create the configuration
		HashMap<Integer, Object> tempHashMap = new HashMap<Integer, Object>();
		tempHashMap.put(2, null);
		configuration = new HashMap<Integer, Object>();
		configuration.put(1, tempHashMap);

		//configure the blackboards
		try {
			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");

			FieldUpdateSubscription statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			serviceStepBBClient.subscribe(statusSubscription);

			equipletStepBBClient = new BlackboardClient(dbData.getIp());
			equipletStepBBClient.setDatabase(dbData.getName());
			equipletStepBBClient.setCollection("EquipletStepsBlackBoard");
			equipletStepBBClient.subscribe(new BasicOperationSubscription(MongoOperation.UPDATE, this));
		} catch (InvalidDBNamespaceException | UnknownHostException | GeneralMongoException e) {
			Logger.log(e);
			doDelete();
		}

		//Start the behaviours
		EvaluateDuration evaluateDurationBehaviour = new EvaluateDuration(this, moduleFactory);
		addBehaviour(evaluateDurationBehaviour);

		FillPlaceholders fillPlaceholdersBehaviour = new FillPlaceholders(this, moduleFactory);
		addBehaviour(fillPlaceholdersBehaviour);

		CheckForModules checkForModules = new CheckForModules(this);
		addBehaviour(checkForModules);
		
		ServiceAgentDied serviceAgentDied = new ServiceAgentDied(this);
		addBehaviour(serviceAgentDied);

		//Register modules
		try {
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] rows = client.executeSelectQuery(Queries.MODULES_PER_EQUIPLET, equipletAgentAID.getLocalName());
			Module module;
			int id;
			for (Row row : rows) {
				id = (int) row.get("module");
				module = moduleFactory.getModuleById(id);
				for (int step : module.isLeadingForSteps()) {
					registerLeadingModule(step, id);
				}
			}
		} catch (KnowledgeException | KeyNotFoundException e1) {
			Logger.log(e1);
			doDelete();
		}

		//Send a message to the serviceAgent that the hardware agent is ready.
		ACLMessage startedMessage = new ACLMessage(ACLMessage.INFORM);
		startedMessage.addReceiver(serviceAgentAID);
		startedMessage.setOntology("InitialisationFinished");
		send(startedMessage);
	}

	/**
	 * Getter for the equipletAgentAID
	 * @return the equipletAgentAID
	 **/
	public AID getEquipletAgentAID() {
		return equipletAgentAID;
	}

	/**
	 * @see Agent#takeDown()
	 */
	@Override
	public void takeDown() {
		try {
			// Clears his own blackboard and removes his subscription on that
			// blackboard.
			equipletStepBBClient.removeDocuments(new BasicDBObject());
			equipletStepBBClient.unsubscribe(new BasicOperationSubscription(MongoOperation.UPDATE, this));
		} catch (InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
		}

		//Send the serviceAgent that he died.
		ACLMessage deadMessage = new ACLMessage(ACLMessage.FAILURE);
		deadMessage.addReceiver(serviceAgentAID);
		deadMessage.setOntology("HardwareAgentDied");
		send(deadMessage);
	}

	/**
	 * Getter for the serviceSteps blackboard client
	 * @return serviceStepBBClient
	 */
	public BlackboardClient getServiceStepsBBClient() {
		return serviceStepBBClient;
	}

	/**
	 * Getter for the equipletSteps blackboard client
	 * @return equipletStepsBBClient
	 */
	public BlackboardClient getEquipletStepsBBClient() {
		return equipletStepBBClient;
	}

	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		switch (entry.getNamespace().split("\\.")[1]) {
		case "ServiceStepsBlackboard":
			switch (operation) {
			case UPDATE:
				//als service op waiting word gezet pak je eerste equipletstep die ook op 
				//waiting moet worden gezet. 
				//ObjectId id = entry.getTargetObjectId();
				try {
					EquipletStep equipletStep = new EquipletStep((BasicDBObject) equipletStepBBClient.findDocumentById(entry.getTargetObjectId()));
					ServiceStep serviceStep = new ServiceStep((BasicDBObject) serviceStepBBClient.findDocumentById(equipletStep.getServiceStepID()));
					BasicDBObject searchQuery = new BasicDBObject("_id", serviceStep.getId());
					StepStatusCode status = serviceStep.getStatus();
					switch(status){
					case WAITING:
						BasicDBObject statusData = serviceStep.getStatusData();
						
						//equipletStep.setStatus(StepStatusCode.WAITING);
						break;
					default:
						break;
					
					}
				} catch (InvalidDBNamespaceException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (GeneralMongoException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				break;
			default:
				break;
			}
			break;
		case "EquipletStepsBlackboard":
			switch (operation) {
			case UPDATE:
				try {
					EquipletStep equipletStep = new EquipletStep((BasicDBObject) equipletStepBBClient.findDocumentById(entry.getTargetObjectId()));
					ServiceStep serviceStep = new ServiceStep((BasicDBObject) serviceStepBBClient.findDocumentById(equipletStep.getServiceStepID()));
					BasicDBObject searchQuery = new BasicDBObject("_id", serviceStep.getId());
					StepStatusCode status = equipletStep.getStatus();
					switch (status) {
					case IN_PROGRESS:
					case SUSPENDED_OR_WARNING:
					case DONE:
					case ABORTED:
					case FAILED:
						BasicDBObject statusData = serviceStep.getStatusData();
						statusData.putAll((Map<String, Object>) equipletStep.getStatusData());
						BasicDBObject updateQuery = new BasicDBObject("$set", new BasicDBObject("status", status).append("statusData", statusData));
						serviceStepBBClient.updateDocuments(searchQuery, updateQuery);
						break;
					default:
						break;
					}
				} catch (InvalidDBNamespaceException | GeneralMongoException e) {
					// TODO Error no document
					Logger.log(e);
				}
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
	}

	/**
	 * @see ModuleUpdateListener#onModuleUpdate(int, Module, Module)
	 */
	@Override
	public void onModuleUpdate(int moduleId, Module oldSoftware, Module newSoftware) {
		//remove old values from HashMap
		for (int step : oldSoftware.isLeadingForSteps()) {
			leadingModules.remove(step);
		}
		//add new values to HashMap
		for (int step : newSoftware.isLeadingForSteps()) {
			leadingModules.put(step, moduleId);
		}
	}

	/**
	 * Getter for the configuration
	 * @return configuration
	 */
	public HashMap<Integer, Object> getConfiguration() {
		return configuration;
	}
}
