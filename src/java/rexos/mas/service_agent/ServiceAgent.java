/**
 * @file ServiceAgent.java
 * @brief This agent manages services and oversees generation and scheduling of serviceSteps. It also handles
 *        communication with the logistics agent.
 * @date Created: 5 apr. 2013
 * 
 * @author Peter Bonnema
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 *          the following conditions are met:
 *          - Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *          following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *          following disclaimer in the documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be
 *          used to endorse or promote products derived from this software without specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package rexos.mas.service_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.List;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.FieldUpdateSubscription;
import rexos.libraries.blackboard_client.FieldUpdateSubscription.MongoUpdateLogOperation;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;
import rexos.libraries.log.Logger;
import rexos.mas.data.DbData;
import rexos.mas.data.ProductStep;
import rexos.mas.data.StepStatusCode;
import rexos.mas.service_agent.behaviours.CanDoProductStep;
import rexos.mas.service_agent.behaviours.GetProductStepDuration;
import rexos.mas.service_agent.behaviours.InitialisationFinished;
import rexos.mas.service_agent.behaviours.ScheduleStep;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

/**
 * This agent manages services and oversees generation and scheduling of
 * serviceSteps. It also handles communication with the logistics agent.
 * 
 * @author Peter Bonnema
 * 
 */
public class ServiceAgent extends Agent implements BlackboardSubscriber {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = -5730473679881365598L;

	/**
	 * @var BlackboardClient productStepBBClient
	 *      The BlackboardClient used to interact with the product steps
	 *      blackboard.
	 */
	private BlackboardClient productStepBBClient;

	/**
	 * @var BlackboardClient serviceStepBBClient
	 *      The BlackboardClient used to interact with the service steps
	 *      blackboard.
	 */
	private BlackboardClient serviceStepBBClient;

	/**
	 * @var FieldUpdateSubscription statusSubscription
	 *      The subscription object used to subscribe this agent on a blackboard
	 *      client to field updates on a blackboard.
	 */
	private FieldUpdateSubscription statusSubscription;

	/**
	 * @var DbData dbData
	 *      Contains connection information for mongoDB database containing the
	 *      productstep and servicestep blackboards.
	 */
	private DbData dbData;

	/**
	 * @var AID equipletAgentAID
	 *      The AID of the equiplet agent of this equiplet.
	 */
	private AID equipletAgentAID;

	/**
	 * @var AID hardwareAgentAID
	 *      The AID of the hardware agent of this equiplet.
	 */
	private AID hardwareAgentAID;

	/**
	 * @var AID logisticsAID
	 *      The AID of the logistics agent.
	 */
	private AID logisticsAID;

	/**
	 * @var ServiceFactory serviceFactory
	 *      The serviceFactory used to create instances of Service.
	 */
	private ServiceFactory serviceFactory;

	/**
	 * @var HashMap<String, Service> convIdServiceMapping
	 *      This maps conversation id's with service objects.
	 */
	private HashMap<String, Service> convIdServiceMapping;

	/**
	 * Initializes the agent. This includes creating and starting the hardware agent, creating and configuring two
	 * blackboard clients, subscribing to status updates and adding all behaviours.
	 */
	@Override
	public void setup() {
		Logger.log("I spawned as a service agent.");

		// handle arguments given to this agent
		Object[] args = getArguments();
		if(args != null && args.length > 0) {
			dbData = (DbData) args[0];
			equipletAgentAID = (AID) args[1];
			logisticsAID = (AID) args[2];
		}

		// Create a hardware agent for this equiplet
		Object[] arguments = new Object[] {
				dbData, equipletAgentAID, getAID()
		};
		try {
			AgentController hardwareAgentCnt =
					getContainerController().createNewAgent(equipletAgentAID.getLocalName() + "-hardwareAgent",
							"rexos.mas.hardware_agent.HardwareAgent", arguments);
			hardwareAgentCnt.start();
			hardwareAgentAID = new AID(hardwareAgentCnt.getName(), AID.ISGUID);
		} catch(StaleProxyException e) {
			Logger.log(e);
			doDelete();
		}

		try {
			// create blackboard clients, configure them and subscribe to status
			// changes of any steps
			productStepBBClient = new BlackboardClient(dbData.getIp());
			serviceStepBBClient = new BlackboardClient(dbData.getIp());
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);

			productStepBBClient.setDatabase(dbData.getName());
			productStepBBClient.setCollection("ProductStepsBlackBoard");
			// Needs to react on state changes of production steps to WAITING
			productStepBBClient.subscribe(statusSubscription);

			serviceStepBBClient.setDatabase(dbData.getName());
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");
			// Needs to react on status changes
			serviceStepBBClient.subscribe(statusSubscription);
			serviceStepBBClient.removeDocuments(new BasicDBObject());
		} catch(UnknownHostException | GeneralMongoException | InvalidDBNamespaceException e) {
			Logger.log(e);
			doDelete();
		}

		convIdServiceMapping = new HashMap<String, Service>();
		serviceFactory = new ServiceFactory(equipletAgentAID.getLocalName());

		// Add behaviours
		addBehaviour(new CanDoProductStep(this, serviceFactory));
		addBehaviour(new GetProductStepDuration(this));
		addBehaviour(new ScheduleStep(this));
		addBehaviour(new InitialisationFinished(this));
	}

	/**
	 * Kill the agent by unsubscribing to status field updates, emptying the service step blackboard and updating the
	 * status of all product steps. It also notifies the equiplet agent and hardware agent that this agent died.
	 */
	//@formatter:off
	@Override
	public void takeDown() {
		Logger.log("ServiceAgent takedown");
		
		productStepBBClient.unsubscribe(statusSubscription);
		serviceStepBBClient.unsubscribe(statusSubscription);
		
		try {
			List<DBObject> productSteps = productStepBBClient.findDocuments(new BasicDBObject());
			ObjectId id;
			BasicDBObject update, statusData;
			for(DBObject productStep : productSteps) {
				id = (ObjectId) productStep.get("_id");
				statusData = new BasicDBObject("source", "service agent")
						.append("reason", "died")
						.append("log", buildLog(id));
				update = new BasicDBObject("status", StepStatusCode.FAILED.name());
				update.append("statusData", statusData);
				
				productStepBBClient.updateDocuments(new BasicDBObject("_id", id), new BasicDBObject("$set", update));
			}
			
			serviceStepBBClient.removeDocuments(new BasicDBObject());
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
		}

		ACLMessage message = new ACLMessage(ACLMessage.INFORM);
		message.addReceiver(equipletAgentAID);
		message.setOntology("ServiceAgentDied");
		send(message);
	} //@formatter:on

	/**
	 * Maps the specified conversation id with the specified service object.
	 * Once a service object has been created (by the service factory) it's
	 * mapped with the conversation id of the scheduling negotiation of the
	 * corresponding product step. With this mapping behaviours that do not have
	 * a reference to a service object can still get one with
	 * GetServiceForConvId.
	 * 
	 * @param conversationId the conversation id to map the service with.
	 * @param service the service object that the conversation id will be mapped with.
	 */
	public void MapConvIdWithService(String conversationId, Service service) {
		convIdServiceMapping.put(conversationId, service);
	}

	/**
	 * Returns the service object mapped to the specified conversation id.
	 * 
	 * @param conversationId the conversation id to use to get the corresponding service object.
	 * @return the service object mapped to the specified conversation id.
	 */
	public Service GetServiceForConvId(String conversationId) {
		return convIdServiceMapping.get(conversationId);
	}

	/**
	 * Removes the mapping of the specified conversation id with the corresponding service object.
	 * 
	 * @param conversationId
	 * 		The conversationId to remove from the mapping.
	 */
	public void RemoveConvIdServiceMapping(String conversationId) {
		convIdServiceMapping.remove(conversationId);
	}

	/**
	 * This method is called by the blackboard client when certain (or any) CRUD operations are performed on the status
	 * field of a product-/servicestep on a blackboard. For now the service agent just passes the message on to another
	 * agent by updating the status field of the corresponding steps on the other blackboard. (e.g. changes the status
	 * of all service steps to ABORTED when the status of a product step is changed to ABORTED).
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			switch(entry.getNamespace().split("\\.")[1]) {
				case "ProductStepsBlackBoard":
					ProductStep productionStep =
							new ProductStep((BasicDBObject) productStepBBClient.findDocumentById(entry
									.getTargetObjectId()));
					switch(operation) {
						case UPDATE:
							StepStatusCode status = productionStep.getStatus();
							switch(status) {
								case WAITING:
									Logger.log("Service agent - prod.Step %s status set to %s\n",
											productionStep.getId(), status);

									// fetch and sort all serviceSteps
									List<DBObject> dbServiceSteps =
											serviceStepBBClient.findDocuments(new BasicDBObject("productStepId",
													productionStep.getId()));
									ServiceStep[] serviceSteps = new ServiceStep[dbServiceSteps.size()];
									for(int i = 0; i < dbServiceSteps.size(); i++) {
										serviceSteps[i] = new ServiceStep((BasicDBObject) dbServiceSteps.get(i));
									}
									serviceSteps = ServiceStep.sort(serviceSteps);

									Logger.log("Service agent - setting status of serv.Step %s to %s\n",
											serviceSteps[0].getId(), status);

									// update the status of the first serviceStep to WAITING
									serviceStepBBClient.updateDocuments(
											new BasicDBObject("_id", serviceSteps[0].getId()),
											new BasicDBObject("$set", new BasicDBObject("status", status.name())
													.append("statusData", productionStep.getStatusData())));
									break;
								case ABORTED:
									Logger.log("Service agent - prod.Step %s status set to %s\n",
											productionStep.getId(), status);

									Logger.log("Service agent - aboring all serviceSteps of prod.Step\n",
											entry.getTargetObjectId());
									
									//TODO inform LA to cancel part transport
									
									serviceStepBBClient.updateDocuments(
											new BasicDBObject("productStepId", entry.getTargetObjectId()),
											new BasicDBObject("$set", new BasicDBObject("status", status.name())
													.append("statusData", productionStep.getStatusData())));
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
					break;
				case "ServiceStepsBlackBoard":
					ServiceStep serviceStep =
							new ServiceStep((BasicDBObject) serviceStepBBClient.findDocumentById(entry
									.getTargetObjectId()));
					ObjectId productStepId = serviceStep.getProductStepId();
					switch(operation) {
						case UPDATE:
							StepStatusCode status = serviceStep.getStatus();
							switch(status) {
								case DELETED:
									Logger.log("Service agent - serv.Step %s status set to %s\n", serviceStep.getId(),
											status);
									productStepBBClient.updateDocuments(
											new BasicDBObject("_id", serviceStep.getProductStepId()),
											new BasicDBObject("$set", new BasicDBObject("status",
													StepStatusCode.DELETED.name()).append("statusData.log",
													buildLog(serviceStep.getProductStepId()))));
									serviceStepBBClient.removeDocuments(new BasicDBObject("_id", serviceStep
											.getId()));
									break;
								case DONE:
									Logger.log("Service agent - serv.Step %s status set to %s\n", serviceStep.getId(),
											status);

									if(serviceStep.getNextStep() != null) {
										Logger.log("Service agent - setting status of next serv.Step %s to %s\n",
												serviceStep.getNextStep(), StepStatusCode.WAITING);
										serviceStepBBClient.updateDocuments(
												new BasicDBObject("_id", serviceStep.getNextStep()),
												new BasicDBObject("$set", new BasicDBObject("status",
														StepStatusCode.WAITING.name())));
										break;
									}

									// fetch and sort all serviceSteps
									List<DBObject> dbServiceSteps =
											serviceStepBBClient.findDocuments(new BasicDBObject("productStepId",
													productStepId));
									ServiceStep[] serviceSteps = new ServiceStep[dbServiceSteps.size()];
									for(int i = 0; i < dbServiceSteps.size(); i++) {
										serviceSteps[i] = new ServiceStep((BasicDBObject) dbServiceSteps.get(i));
									}
									serviceSteps = ServiceStep.sort(serviceSteps);

									// append all serviceSteps to the log
									BasicDBObject log = new BasicDBObject();
									for(int i = 0; i < serviceSteps.length; i++) {
										log.append("step" + i, serviceSteps[i].toBasicDBObject());
									}

									Logger.log("Service agent - saving log in prod.Step %s\n%s\n", productStepId, log);

									// save the log in the productStep
									productStepBBClient.updateDocuments(
											new BasicDBObject("_id", productStepId),
											new BasicDBObject("$set", new BasicDBObject("status", status.name())
													.append("statusData", log)));
									break;
								case IN_PROGRESS:
								case SUSPENDED_OR_WARNING:
								case FAILED:
									Logger.log("Service agent - serv.Step %s status set to %s\n", serviceStep.getId(),
											status);
									Logger.log("Service agent - setting status of prod.Step %s to %s\n", productStepId,
											status);
									productStepBBClient.updateDocuments(
											new BasicDBObject("_id", productStepId),
											new BasicDBObject("$set", new BasicDBObject("status", status.name())
													.append("statusData", serviceStep.getStatusData())));
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
			doDelete();
		}
	}

	/**
	 * Function for building the log of the given productStep.
	 * 
	 * @param productStep the productStep to build the log for.
	 * 
	 * @return the log as a BasicDBObject
	 */
	public BasicDBObject buildLog(ObjectId productStep) {
		BasicDBObject log = new BasicDBObject();
		List<DBObject> dbServiceSteps;
		try {
			dbServiceSteps = serviceStepBBClient.findDocuments(new BasicDBObject("serviceStepID", productStep));

			ServiceStep[] serviceSteps = new ServiceStep[dbServiceSteps.size()];
			for(int i = 0; i < dbServiceSteps.size(); i++) {
				serviceSteps[i] = new ServiceStep((BasicDBObject) dbServiceSteps.get(i));
			}
			serviceSteps = ServiceStep.sort(serviceSteps);

			// append all serviceSteps to the log
			for(int i = 0; i < serviceSteps.length; i++) {
				log.append("step" + i, serviceSteps[i].toBasicDBObject());
			}
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
		}
		return log;
	}

	/**
	 * the DbData containing connection info for the blackboard mongoDB database.
	 * 
	 * @return the DbData containing connection info for the blackboard mongoDB database.
	 */
	public DbData getDbData() {
		return dbData;
	}

	/**
	 * Returns the AID of the equipletAgent of this equiplet.
	 * 
	 * @return the AID of the equipletAgent of this equiplet.
	 */
	public AID getEquipletAgentAID() {
		return equipletAgentAID;
	}

	/**
	 * Returns the AID of the logisticsAgent.
	 * 
	 * @return the AID of the logisticsAgent.
	 */
	public AID getLogisticsAID() {
		return logisticsAID;
	}

	/**
	 * Returns the AID of the hardwareAgent of this equiplet.
	 * 
	 * @return the AID of the hardwareAgent of this equiplet.
	 */
	public AID getHardwareAgentAID() {
		return hardwareAgentAID;
	}

	/**
	 * Returns the productSteps blackboard client.
	 * 
	 * @return the productSteps blackboard client.
	 */
	public BlackboardClient getProductStepBBClient() {
		return productStepBBClient;
	}

	/**
	 * Returns the serviceSteps blackboard client.
	 * 
	 * @return the serviceSteps blackboard client.
	 */
	public BlackboardClient getServiceStepBBClient() {
		return serviceStepBBClient;
	}
}
