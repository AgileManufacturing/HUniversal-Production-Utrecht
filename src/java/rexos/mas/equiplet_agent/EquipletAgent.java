/**
 * @file rexos/mas/equiplet_agent/EquipletAgent.java
 * @brief Provides an equiplet agent that communicates with product agents and
 *        with its own service agent.
 * @date Created: 2013-04-02
 * 
 * @author Hessel Meulenbeld
 * @author Thierry Gerritse
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

package rexos.mas.equiplet_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.bson.types.ObjectId;

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
import rexos.mas.data.EquipletState;
import rexos.mas.data.EquipletStateEntry;
import rexos.mas.data.ProductStep;
import rexos.mas.data.ScheduleData;
import rexos.mas.data.StepStatusCode;
import rexos.mas.equiplet_agent.behaviours.AbortStep;
import rexos.mas.equiplet_agent.behaviours.InitialisationFinished;
import rexos.mas.equiplet_agent.behaviours.ServiceAgentDied;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

/**
 * EquipletAgent that communicates with product agents and with its own service agent.
 **/
public class EquipletAgent extends Agent implements BlackboardSubscriber {
	/**
	 * @var long serialVersionUID
	 *      The serial version UID.
	 */
	private static final long serialVersionUID = 1L;

	private int equipletId;

	/**
	 * @var AID serviceAgent
	 *      AID of the serviceAgent connected to this EquipletAgent.
	 */
	private AID serviceAgent;

	/**
	 * @var String collectiveDbIp
	 *      IP of the collective database.
	 */
	private String collectiveDbIp = "145.89.191.131";

	/**
	 * @var int collectiveDbPort
	 *      Port number of the collective database.
	 */
	private int collectiveDbPort = 27017;

	/**
	 * @var String collectiveDbName
	 *      Name of the collective database.
	 */
	private String collectiveDbName = "CollectiveDb";

	/**
	 * @var String equipletDirectoryName
	 *      Name of the collection containing the equipletDirectory.
	 */
	private String equipletDirectoryName = "EquipletDirectory";

	/**
	 * @var String timeDataName
	 *      Name of the collection containing the timeData.
	 */
	private String timeDataName = "TimeData";

	/**
	 * @var String equipletDbIp
	 *      IP of the equiplet database.
	 */
	private String equipletDbIp = "localhost";

	/**
	 * @var int equipletDbPort
	 *      Port number of the equiplet database.
	 */
	private int equipletDbPort = 27017;

	/**
	 * @var String equipletDbName
	 *      Name of the equiplet database.
	 */
	private String equipletDbName = "";

	/**
	 * @var String productStepsName
	 *      Name of the collection containing the productSteps.
	 */
	private String productStepsName = "ProductStepsBlackBoard";

	/**
	 * @var BlackboardClient collectiveBBClient
	 *      Object for communication with the collective blackboard.
	 */
	private BlackboardClient collectiveBBClient;

	/**
	 * @var BlackboardClient productStepBBClient
	 *      Object for communication with the equiplet blackboard.
	 */
	private BlackboardClient productStepBBClient;

	/**
	 * @var BlackboardClient stateStepBBClient
	 *      The blackboard client for the state blackboard.
	 */
	private BlackboardClient stateBBClient;
	private BlackboardClient desiredStateBBClient;

	private FieldUpdateSubscription statusSubscription;

	private FieldUpdateSubscription modeUpdateSubscription;

	/**
	 * @var ArrayList<Integer> capabilities
	 *      List with all the capabilities of this equiplet.
	 */
	private ArrayList<Integer> capabilities;

	/**
	 * @var HashMap<String, ObjectId> communicationTable
	 *      Table with the combinations conversationID and ObjectId.
	 */
	private HashMap<String, ObjectId> communicationTable;

	/**
	 * @var NextProductStepTimer timer
	 *      Timer used to trigger when the next used time slot is ready to
	 *      start.
	 */
	private NextProductStepTimer timer;

	/**
	 * @var ObjectId nextProductStep
	 *      The next product step.
	 */
	private ObjectId nextProductStep;

	/**
	 * @var DbData dbData
	 *      The dbData of the equipletAgent.
	 */
	private DbData dbData;

	/**
	 * Setup function for the equipletAgent. Configures the IP and database name
	 * of the equiplet. Gets its capabilities from the arguments. Creates its
	 * service agent. Makes connections with the BlackboardCLients and
	 * subscribes on changes on the
	 * status field. Puts its capabilities on the equipletDirectory blackboard.
	 * Gets the time data from the blackboard. Initializes the Timer objects.
	 * Starts its behaviours.
	 */
	@Override
	public void setup() {
		Logger.log("I spawned as a equiplet agent.");
		// gets his IP and sets the equiplet blackboard IP.
		try {
			InetAddress IP = InetAddress.getLocalHost();
			equipletDbIp = IP.getHostAddress();
		} catch(UnknownHostException e) {
			Logger.log(e);
		}

		equipletDbName = getAID().getLocalName();
		communicationTable = new HashMap<String, ObjectId>();
		try {
			Object[] args = getArguments();
			AID logisticsAgent = null;
			if(args != null && args.length > 0) {
				logisticsAgent = (AID) args[0];
			}

			capabilities = new ArrayList<Integer>();

			// Register modules
			try {
				KnowledgeDBClient client = KnowledgeDBClient.getClient();
				Row[] rows = client.executeSelectQuery(Queries.POSSIBLE_STEPS_PER_EQUIPLET, getAID().getLocalName());
				for(Row row : rows) {
					capabilities.add((int) row.get("id"));
				}
			} catch(KnowledgeException | KeyNotFoundException e1) {
				doDelete();
				Logger.log(e1);
			}
			Logger.log("%s %s%n", capabilities, equipletDbName);

			dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);

			// TODO register this equiplet on the knowledge db and add the equipletId to the arguments for the SA
			// creates his service agent.
			equipletId = 1;

			Object[] arguments = new Object[] {
					dbData, getAID(), logisticsAgent
			};
			AgentController serviceAgentCnt =
					getContainerController().createNewAgent(getLocalName() + "-serviceAgent",
							"rexos.mas.service_agent.ServiceAgent", arguments);
			serviceAgentCnt.start();
			serviceAgent = new AID(serviceAgentCnt.getName(), AID.ISGUID);

			// makes connection with the collective blackboard.
			collectiveBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			collectiveBBClient.setDatabase(collectiveDbName);
			collectiveBBClient.setCollection(equipletDirectoryName);

			// makes connection with the equiplet blackboard.
			productStepBBClient = new BlackboardClient(equipletDbIp, equipletDbPort);
			productStepBBClient.setDatabase(equipletDbName);
			productStepBBClient.setCollection(productStepsName);
			productStepBBClient.removeDocuments(new BasicDBObject());

			// subscribes on changes of the status field on the equiplet blackboard.
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			productStepBBClient.subscribe(statusSubscription);

			stateBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			stateBBClient.setDatabase("StateBlackboard");
			stateBBClient.setCollection("equipletState");

			modeUpdateSubscription = new FieldUpdateSubscription("mode", this);
			modeUpdateSubscription.addOperation(MongoUpdateLogOperation.SET);
			stateBBClient.subscribe(modeUpdateSubscription);

			desiredStateBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			desiredStateBBClient.setDatabase("StateBlackboard");
			desiredStateBBClient.setCollection("EquipletCommands");

			// gets the timedata for synchronizing from the collective blackboard.
			collectiveBBClient.setCollection(timeDataName);
			BasicDBObject timeData = (BasicDBObject) collectiveBBClient.findDocuments(new BasicDBObject()).get(0);

			// initiates the timer to the next product step.
			timer =
					new NextProductStepTimer(timeData.getLong("firstTimeSlot"), timeData.getInt("timeSlotLength"), this);

			collectiveBBClient.setCollection(equipletDirectoryName);
		} catch(GeneralMongoException | InvalidDBNamespaceException | UnknownHostException | StaleProxyException e) {
			Logger.log(e);
			doDelete();
		}

		// starts the behaviour for receiving message when the Service Agent dies.
		addBehaviour(new ServiceAgentDied(this));

		// starts the behaviour for receiving message initialization finished.
		addBehaviour(new InitialisationFinished(this));

		// starts the behaviour for receiving message initialization finished.
		addBehaviour(new AbortStep(this));
	}

	/**
	 * Takedown function for the equipletAgent. Removes itself from the
	 * equipletDirectory. Informs the productAgents who have planned a
	 * productStep on its blackboard of its dead. Removes its database.
	 */
	@Override
	public void takeDown() {
		try {
			// Removes himself from the collective blackboard equiplet directory.
			collectiveBBClient.removeDocuments(new BasicDBObject("AID", getAID().getName()));

			// Messages all the product agents that he died.
			for(DBObject object : productStepBBClient.findDocuments(new BasicDBObject())) {
				ACLMessage responseMessage = new ACLMessage(ACLMessage.FAILURE);
				responseMessage.addReceiver(new AID(object.get("productAgentId").toString(), AID.ISGUID));
				responseMessage.setOntology("EquipletAgentDied");
				responseMessage.setContentObject((BasicDBObject) object.get("statusData"));
				send(responseMessage);
			}

			// Clears his own blackboard and removes his subscription on that blackboard.
			productStepBBClient.removeDocuments(new BasicDBObject());
			productStepBBClient.unsubscribe(statusSubscription);
		} catch(InvalidDBNamespaceException | GeneralMongoException | IOException e) {
			Logger.log(e);
		}

		ACLMessage deadMessage = new ACLMessage(ACLMessage.FAILURE);
		deadMessage.addReceiver(serviceAgent);
		deadMessage.setOntology("EquipletAgentDied");
		send(deadMessage);
	}

	public void cancelAllStepsForProductStep(ObjectId productStepId, String reason) {
		try {
			productStepBBClient.updateDocuments(
					new BasicDBObject("_id", productStepId),
					new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.ABORTED.name()).append(
							"statusData", new BasicDBObject("reason", reason))));
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			e.printStackTrace();
		}
	}

	/**
	 * onMessage function for the equipletAgent. Listens to updates of the
	 * blackboard clients and handles them.
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			switch(entry.getNamespace().split("\\.")[1]) {
				case "ProductStepsBlackBoard":
					// Get the productstep.
					ObjectId id = entry.getTargetObjectId();
					ProductStep productStep = new ProductStep((BasicDBObject) productStepBBClient.findDocumentById(id));

					// Gets the conversationId
					String conversationId = getConversationId(id);

					// Create the responseMessage
					ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
					responseMessage.addReceiver(productStep.getProductAgentId());
					responseMessage.setConversationId(conversationId);

					Logger.log("Equiplet agent - status update: " + productStep.getStatus().toString());
					switch(productStep.getStatus()) {
					// Depending on the changed status fills in the responseMessage and sends it to the product agent.
						case PLANNED:
							try {
								// If the start time of the newly planned productStep is earlier then the next used time
								// slot make it the next used timeslot.
								nextProductStep = productStep.getId();
								ScheduleData scheduleData = productStep.getScheduleData();
								if(timer.getNextUsedTimeSlot() == 0
										|| scheduleData.getStartTime() < timer.getNextUsedTimeSlot()) {
									timer.setNextUsedTimeSlot(scheduleData.getStartTime());
								}

								responseMessage.setOntology("Planned");
								responseMessage.setContentObject(scheduleData.getStartTime());

								// TODO: after testing delete below
								// addBehaviour(new WakerBehaviour(this, 75){
								//
								// /**
								// *
								// */
								// private static final long serialVersionUID = 1L;
								//
								// protected void onWake(){
								//
								// ACLMessage cancelMessage = new ACLMessage(ACLMessage.CANCEL);
								// cancelMessage.addReceiver(getAID());
								// cancelMessage.setOntology("AbortStep");
								// cancelMessage.setConversationId(getConversationId(nextProductStep));
								// send(cancelMessage);
								//
								// Logger.log("Equiplet agent - sending message %s%n",
								// ACLMessage.getPerformative(cancelMessage.getPerformative()));
								// }
								// });
								// TODO: after testing delete above

							} catch(IOException e) {
								responseMessage.setPerformative(ACLMessage.FAILURE);
								responseMessage.setContent("An error occured in the planning/please reschedule");
								Logger.log(e);
							}
							break;
						case WAITING:
						case IN_PROGRESS:
						case FAILED:
						case SUSPENDED_OR_WARNING:
							setDesiredEquipletState(EquipletState.STANDBY);

							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							break;
						case DONE:
							setDesiredEquipletState(EquipletState.STANDBY);

							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							productStep.setStatus(StepStatusCode.DONE);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							productStepBBClient.removeDocuments(new BasicDBObject("_id", productStep.getId()));
							break;
						case DELETED:
							setDesiredEquipletState(EquipletState.STANDBY);

							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							productStepBBClient.removeDocuments(new BasicDBObject("_id", productStep.getId()));
							break;
						default:
							break;
					}
					Logger.log("Equiplet agent - sending message %s%n",
							ACLMessage.getPerformative(responseMessage.getPerformative()));
					send(responseMessage);
					break;
				case "stateBlackboard":
					EquipletStateEntry stateEntry =
							new EquipletStateEntry((BasicDBObject) stateBBClient.findDocumentById(entry
									.getTargetObjectId()));
					switch(stateEntry.getEquipletMode()) {
					// TODO handle error stuff
						case ERROR:
							break;
						case CRITICAL_ERROR:
							break;
						case EMERGENCY_STOP:
							break;
						case LOCK:
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
		} catch(GeneralMongoException | InvalidDBNamespaceException | IOException e) {
			// TODO handle error
			Logger.log(e);
		}
	}

	public EquipletStateEntry getEquipletStateEntry() throws InvalidDBNamespaceException, GeneralMongoException {
		List<DBObject> equipletStates = stateBBClient.findDocuments(new BasicDBObject("id", equipletId));
		return new EquipletStateEntry((BasicDBObject) equipletStates.get(0));
	}

	public void setDesiredEquipletState(EquipletState state) throws InvalidDBNamespaceException, GeneralMongoException {
//		desiredStateBBClient.updateDocuments(new BasicDBObject("id", equipletId), new BasicDBObject("$set", new BasicDBObject(
//				"desiredState", state.getValue())));
		desiredStateBBClient.updateDocuments(new BasicDBObject("name", getLocalName()), new BasicDBObject("$set", new BasicDBObject(
				"desiredState", state.getValue())));
	}

	/**
	 * Getter for the collectiveBBClient
	 * 
	 * @return the collectiveBBClient.
	 */
	public BlackboardClient getCollectiveBBClient() {
		return collectiveBBClient;
	}

	/**
	 * Getter for the productStepBBClient
	 * 
	 * @return the productStepBBClient.
	 */
	public BlackboardClient getProductStepBBClient() {
		return productStepBBClient;
	}

	/**
	 * @return the stateBBClient
	 */
	public BlackboardClient getStateBBClient() {
		return stateBBClient;
	}

	/**
	 * Function for adding a new relation between conversationId and objectId
	 * 
	 * @param conversationId
	 *            the conversationId in the new relation.
	 * @param objectId
	 *            the objectId in the new relation.
	 */
	public void addCommunicationRelation(String conversationId, ObjectId objectId) {
		communicationTable.put(conversationId, objectId);
	}

	/**
	 * Getter for getting the objectId by a conversationId.
	 * 
	 * @param conversationId
	 *            the conversationId of which the related objectId is needed.
	 * @return ObjectId for the given conversationId.
	 */
	public ObjectId getRelatedObjectId(String conversationId) {
		return communicationTable.get(conversationId);
	}

	/**
	 * Getter for getting the related conversationId for the given ObjectId.
	 * 
	 * @param productStepEntry
	 *            the ObjectId for which the related conversationId is needed.
	 * @return the related conversationId or null if the relation does not
	 *         exist.
	 */
	public String getConversationId(ObjectId productStepEntry) {
		String conversationId = null;
		if(communicationTable.containsValue(productStepEntry)) {
			for(Entry<String, ObjectId> tableEntry : communicationTable.entrySet()) {
				if(tableEntry.getValue().equals(productStepEntry)) {
					conversationId = tableEntry.getKey();
					break;
				}
			}
		}
		return conversationId;
	}

	/**
	 * Getter for the service agent from this equiplet agent.
	 * 
	 * @return the serviceAgent.
	 */
	public AID getServiceAgent() {
		return serviceAgent;
	}

	/**
	 * Getter for the timer that handles the next product step.
	 * 
	 * @return the timer.
	 */
	public NextProductStepTimer getTimer() {
		return timer;
	}

	/**
	 * Getter for the next product step.
	 * 
	 * @return the nextProductStep
	 */
	public ObjectId getNextProductStep() {
		return nextProductStep;
	}

	/**
	 * Setter for the next product step
	 * 
	 * @param nextProductStep The new next product step
	 */
	public void setNextProductStep(ObjectId nextProductStep) {
		this.nextProductStep = nextProductStep;
	}

	/**
	 * Returns the capabilities (i.e. product step ids of steps it can perform)
	 * for this equiplet.
	 * 
	 * @return The capabilities for this equiplet.
	 * 
	 */
	public ArrayList<Integer> getCapabilities() {
		return capabilities;
	}

	/**
	 * Returns the details for this Equiplet's database.
	 * 
	 * @return DbData object containing the details for this Equiplet's
	 *         database.
	 * 
	 */
	public DbData getDbData() {
		return dbData;
	}
}
