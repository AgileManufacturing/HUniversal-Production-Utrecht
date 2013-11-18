/**
 * @file rexos/mas/hardware_agent/HardwareAgent.java
 * @brief Provides an Hardware agent that communicates with Service agents and
 *        its own modules.
 * @date Created: 12-04-13
 * 
 * @author Thierry Gerritse
 * @author Hessel Meulenbeld
 * @author Wouter Veen
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

package agents.hardware_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.BlackboardSubscriber;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.MongoOperation;
import libraries.blackboard_client.data_classes.OplogEntry;
import libraries.blackboard_client.data_classes.FieldUpdateSubscription.MongoUpdateLogOperation;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Queries;
import libraries.knowledgedb_client.Row;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data_classes.DbData;
import agents.data_classes.StepStatusCode;
import agents.hardware_agent.behaviours.RequiredModulesPresent;
import agents.hardware_agent.behaviours.EvaluateDuration;
import agents.hardware_agent.behaviours.FillPlaceholders;
import agents.hardware_agent.behaviours.ServiceAgentDied;
import agents.service_agent.ServiceStep;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import configuration.Configuration;
import configuration.ConfigurationFiles;

/**
 * HardwareAgent that communicates with the service agent and creates the messages for the Hardware layer.
 */
public class HardwareAgent extends Agent implements BlackboardSubscriber, ModuleUpdateListener {
	/**
	 * @var long serialVersionUID
	 *      The serial version UID.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var BlackboardClient serviceStepBBClient
	 *      The blackboard client for the serviceStep blackboard.
	 */
	private BlackboardClient serviceStepBBClient;

	/**
	 * @var BlackboardClient equipletStepBBClient
	 *      The blackboard client for the equipletStep blackboard.
	 */
	private BlackboardClient equipletStepBBClient;

	/**
	 * @var DbData dbData
	 *      The DbData of this equiplet.
	 */
	private DbData dbData;

	/**
	 * @var HashMap<Integer, Integer> leadingModules
	 *      A HashMap containing the leadingModules per step.
	 */
	private HashMap<Integer, Integer> leadingModules;

	/**
	 * @var ModuleFactory moduleFactory
	 *      The moduleFactory of this agent.
	 */
	private ModuleFactory moduleFactory;

	/**
	 * @var AID equipletAgentAID
	 *      The AID of the equipletAgent.
	 */
	private AID equipletAgentAID;

	/**
	 * @var AID serviceAgentAID
	 *      The AID of the serviceAgent.
	 */
	private AID serviceAgentAID;

	/**
	 * @var HashMap<Integer, Object> configuration
	 *      The configuration of this
	 *      agent.
	 */
	private HashMap<Integer, HashMap> configuration;

	private FieldUpdateSubscription stepStatusSubscription;
	
	private ArrayList<Behaviour> behaviours;

	/**
	 * Function for registering a leading module.
	 * 
	 * @param serviceId The service id to register for.
	 * @param moduleId The module id to register
	 */
	public void registerLeadingModule(int serviceId, int moduleId) {
		leadingModules.put(serviceId, moduleId);
	}

	/**
	 * Function for getting the leading module for a service id.
	 * 
	 * @param serviceId The service id to get leading module for.
	 * @return the leading module id.
	 */
	public int getLeadingModule(int serviceId) {
		if(!leadingModules.containsKey(serviceId)) {
			return 0;
		}
		return leadingModules.get(serviceId);
	}

	/**
	 * @see Agent#setup()
	 */
	@SuppressWarnings("rawtypes")
	@Override
	public void setup() {
		Logger.log(LogLevel.NOTIFICATION, "" + this.getAID().getLocalName() + " spawned as an hardware agent.");
		leadingModules = new HashMap<Integer, Integer>();

		// gets the modulefactory and subscribes to updates.
		moduleFactory = new ModuleFactory();
		moduleFactory.subscribeToUpdates(this);

		// gets the dbData and AID from the arguments.
		Object[] args = getArguments();
		if(args != null && args.length > 0) {
			dbData = (DbData) args[0];
			equipletAgentAID = (AID) args[1];
			serviceAgentAID = (AID) args[2];
		}

		// create the configuration
		HashMap<Integer, Object> deltaRobotConfiguration = new HashMap<Integer, Object>();
		deltaRobotConfiguration.put(2, null);
		deltaRobotConfiguration.put(3, null);
		configuration = new HashMap<Integer, HashMap>();
		configuration.put(1, deltaRobotConfiguration);
		
		behaviours = new ArrayList<Behaviour>();

		// configure the blackboards
		try {
			// Send a message to the serviceAgent that the hardware agent is ready.
			ACLMessage startedMessage = new ACLMessage(ACLMessage.INFORM);
			startedMessage.addReceiver(serviceAgentAID);
			startedMessage.setOntology("InitialisationFinished");
			send(startedMessage);

			stepStatusSubscription = new FieldUpdateSubscription("status", this);
			stepStatusSubscription.addOperation(MongoUpdateLogOperation.SET);

			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ServiceStepsBlackBoardName", equipletAgentAID.getLocalName()));
			serviceStepBBClient.subscribe(stepStatusSubscription);

			equipletStepBBClient = new BlackboardClient(dbData.getIp());
			equipletStepBBClient.setDatabase(dbData.getName());
			equipletStepBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "EquipletStepsBlackBoardName", equipletAgentAID.getLocalName()));
			equipletStepBBClient.subscribe(stepStatusSubscription);

			equipletStepBBClient.removeDocuments(new BasicDBObject());
		}
		catch(InvalidDBNamespaceException | UnknownHostException | GeneralMongoException e) 
		{
			Logger.log(LogLevel.ERROR, "", e);
			doDelete();
		}

		// Start the evaluateDurationBehaviour
		addBehaviour(new EvaluateDuration(this, moduleFactory));

		// Start the fillPlaceholdersBehaviour
		addBehaviour(new FillPlaceholders(this, moduleFactory));

		// Start the checkForModules
		addBehaviour(new RequiredModulesPresent(this));

		// Start the serviceAgentDied
		addBehaviour(new ServiceAgentDied(this));

		// Get the modules for the equiplet and register the modules
		try {
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			Row[] rows = client.executeSelectQuery(Queries.MODULES_PER_EQUIPLET, equipletAgentAID.getLocalName());
			Module module;
			int id;
			for(Row row : rows) {
				id = (int) row.get("module");
				module = moduleFactory.getModuleById(id);
				for(int step : module.isLeadingForServices()) {
					registerLeadingModule(step, id);
				}
			}
		}
		catch(KnowledgeException | KeyNotFoundException e) 
		{
			Logger.log(LogLevel.ERROR, "", e);
			doDelete();
		}
	}

	/**
	 * @see Agent#takeDown()
	 */
	@Override
	public void takeDown() {
		try {
			// Clears his own blackboard and removes his subscription on that blackboard.
			for(DBObject object : serviceStepBBClient.findDocuments(new BasicDBObject())) {
				serviceStepBBClient.updateDocuments(new BasicDBObject("_id", object.get("_id")), new BasicDBObject(
						"$set", new BasicDBObject("statusData", buildLog((ObjectId) object.get("_id")))));
			}

			equipletStepBBClient.removeDocuments(new BasicDBObject());
			equipletStepBBClient.unsubscribe(stepStatusSubscription);
			serviceStepBBClient.unsubscribe(stepStatusSubscription);
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}

		// Send the serviceAgent that he died.
		ACLMessage deadMessage = new ACLMessage(ACLMessage.FAILURE);
		deadMessage.addReceiver(serviceAgentAID);
		deadMessage.setOntology("HardwareAgentDied");
		send(deadMessage);
	}

	public void cancelAllStepsForServiceStep(ObjectId serviceStepId, String reason) {
		try {
			for(Behaviour behaviour : behaviours) {
				removeBehaviour(behaviour);
			}
			
			serviceStepBBClient.updateDocuments(
					new BasicDBObject("_id", serviceStepId),
					new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.DELETED.name()).append(
							"statusData", new BasicDBObject("reason", reason).append("log", buildLog(serviceStepId)))));
			equipletStepBBClient.removeDocuments(new BasicDBObject("serviceStepID", serviceStepId));
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
	}

	/**
	 * @see BlackboardSubscriber#onMessage(MongoOperation, OplogEntry)
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			DBObject dbObject;
			switch(entry.getNamespace().split("\\.")[1]) {
				case "ServiceStepsBlackBoard":
					dbObject = serviceStepBBClient.findDocumentById(entry.getTargetObjectId());
					if(dbObject != null) {
						ServiceStep serviceStep = new ServiceStep((BasicDBObject) dbObject);
						StepStatusCode status = serviceStep.getServiceStepStatus();
						Logger.log(LogLevel.DEBUG, "serv.Step status set to: %s%n", status);
						
						switch(status) {
							case ABORTED:
								cancelAllStepsForServiceStep(serviceStep.getId(), serviceStep.getStatusData()
										.getString("reason"));
								serviceStepBBClient.updateDocuments(
										new BasicDBObject("_id", serviceStep.getId()),
										new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.DELETED
												.name()).append("statusData.log", buildLog(serviceStep.getId()))));
								equipletStepBBClient.removeDocuments(new BasicDBObject("serviceStepID", serviceStep
										.getId()));

								break;
							case PLANNED:
								equipletStepBBClient.updateDocuments(
										new BasicDBObject("serviceStepID", serviceStep.getId()), new BasicDBObject(
												"$set", new BasicDBObject("status", status.name())));
								break;
							case WAITING:
								List<DBObject> dbObjects =
										equipletStepBBClient.findDocuments(new BasicDBObject("serviceStepID",
												serviceStep.getId()));
								if(dbObjects.size() > 0) {
									EquipletStep[] unsortedSteps = new EquipletStep[dbObjects.size()];
									for(int i = 0; i < dbObjects.size(); i++) {
										unsortedSteps[i] = new EquipletStep((BasicDBObject) dbObjects.get(i));
									}
									ObjectId id = EquipletStep.sort(unsortedSteps)[0].getId();

									equipletStepBBClient.updateDocuments(new BasicDBObject("_id", id),
											new BasicDBObject("$set", new BasicDBObject("status", status.name())));
								}
								break;
							default:
								break;
						}
					}
					break;
				case "EquipletStepsBlackBoard":
					dbObject = equipletStepBBClient.findDocumentById(entry.getTargetObjectId());
					if(dbObject != null) {
						EquipletStep equipletStep = new EquipletStep((BasicDBObject) dbObject);
						ServiceStep serviceStep =
								new ServiceStep((BasicDBObject) serviceStepBBClient.findDocumentById(equipletStep
										.getServiceStepID()));
						BasicDBObject searchQuery = new BasicDBObject("_id", serviceStep.getId());
						StepStatusCode status = equipletStep.getEquipletStepStatus();
						
						Logger.log(LogLevel.DEBUG, "equip.Step no: %s%n status set to: %s%n", equipletStep.getId(), status);
						
						switch(status) {
							case DONE:
								if(equipletStep.getNextEquipletStep() == null) {
									Logger.log(LogLevel.DEBUG, "saving log in serv.Step %s\n%s\n",
											serviceStep.getId(), buildLog(serviceStep.getId()));

									serviceStepBBClient.updateDocuments(
											new BasicDBObject("_id", serviceStep.getId()),
											new BasicDBObject("$set", new BasicDBObject("statusData",
													buildLog(serviceStep.getId())).append("status",
													StepStatusCode.DONE.name())));
									Logger.log(LogLevel.DEBUG, "setting service step on DONE");
								} else {
									equipletStepBBClient
											.updateDocuments(new BasicDBObject("_id", equipletStep.getNextEquipletStep()),
													new BasicDBObject("$set", new BasicDBObject("status",
															StepStatusCode.WAITING.name())));
								}
								break;
							case IN_PROGRESS:
							case SUSPENDED_OR_WARNING:
							case ABORTED:
							case FAILED:
								BasicDBObject serviceStepStatusData = serviceStep.getStatusData();
								if ( serviceStepStatusData != null && equipletStep.getStatusData() != null){
									serviceStepStatusData.putAll((Map<String, Object>) equipletStep.getStatusData());
								}
								BasicDBObject updateQuery =
												new BasicDBObject("$set", new BasicDBObject("status", status.name()).append(
												"statusData", serviceStepStatusData));
								serviceStepBBClient.updateDocuments(searchQuery, updateQuery);
								break;
							default:
								break;
						}
					}
					break;
				default:
					break;
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
	}

	/**
	 * @see ModuleUpdateListener#onModuleUpdate(int, Module, Module)
	 */
	@Override
	public void onModuleUpdate(int moduleId, Module oldSoftware, Module newSoftware) {
		// remove old values from HashMap
		for(int step : oldSoftware.isLeadingForServices()) {
			leadingModules.remove(step);
		}
		// add new values to HashMap
		for(int step : newSoftware.isLeadingForServices()) {
			leadingModules.put(step, moduleId);
		}
	}

	/**
	 * Function for building the log of the given serviceStep.
	 * 
	 * @param serviceStep the serviceStep to build the log for.
	 * 
	 * @return the log as a BasicDBObject
	 */
	public BasicDBObject buildLog(ObjectId serviceStep) {
		BasicDBObject log = new BasicDBObject();
		List<DBObject> dbEquipletSteps;
		try {
			dbEquipletSteps = equipletStepBBClient.findDocuments(new BasicDBObject("serviceStepID", serviceStep));

			EquipletStep[] equipletSteps = new EquipletStep[dbEquipletSteps.size()];
			for(int i = 0; i < dbEquipletSteps.size(); i++) {
				equipletSteps[i] = new EquipletStep((BasicDBObject) dbEquipletSteps.get(i));
			}
			equipletSteps = EquipletStep.sort(equipletSteps);

			// append all equipletsteps to the log
			for(int i = 0; i < equipletSteps.length; i++) {
				log.append("step" + i, equipletSteps[i].toBasicDBObject());
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(LogLevel.ERROR, "", e);
		}
		return log;
	}
	
	@Override
	public void addBehaviour(Behaviour behaviour) {
		super.addBehaviour(behaviour);
		behaviours.add(behaviour);
	}
	
	@Override
	public void removeBehaviour(Behaviour behaviour) {
		super.removeBehaviour(behaviour);
		behaviours.remove(behaviour);
	}

	/**
	 * Getter for the equipletAgentAID
	 * 
	 * @return the equipletAgentAID
	 **/
	public AID getEquipletAgentAID() {
		return equipletAgentAID;
	}

	/**
	 * Getter for the serviceSteps blackboard client
	 * 
	 * @return serviceStepBBClient
	 */
	public BlackboardClient getServiceStepsBBClient() {
		return serviceStepBBClient;
	}

	/**
	 * Getter for the equipletSteps blackboard client
	 * 
	 * @return equipletStepsBBClient
	 */
	public BlackboardClient getEquipletStepsBBClient() {
		return equipletStepBBClient;
	}

	/**
	 * Getter for the configuration
	 * 
	 * @return configuration
	 */
	@SuppressWarnings("rawtypes")
	public HashMap<Integer, HashMap> getConfiguration() {
		return configuration;
	}
}