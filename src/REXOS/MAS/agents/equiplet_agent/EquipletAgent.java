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

package agents.equiplet_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.BlackboardSubscriber;
import libraries.blackboard_client.FieldUpdateSubscription;
import libraries.blackboard_client.FieldUpdateSubscription.MongoUpdateLogOperation;
import libraries.blackboard_client.GeneralMongoException;
import libraries.blackboard_client.InvalidDBNamespaceException;
import libraries.blackboard_client.MongoOperation;
import libraries.blackboard_client.OplogEntry;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Queries;
import libraries.knowledgedb_client.Row;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

import org.bson.types.ObjectId;

import agents.data.DbData;
import agents.data.EquipletMode;
import agents.data.EquipletState;
import agents.data.EquipletStateEntry;
import agents.data.ProductStep;
import agents.data.ScheduleData;
import agents.data.StepStatusCode;
import agents.equiplet_agent.behaviours.AbortStep;
import agents.equiplet_agent.behaviours.InitialisationFinished;
import agents.equiplet_agent.behaviours.ServiceAgentDied;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;

import configuration.Configuration;
import configuration.ConfigurationFiles;

/**
 * EquipletAgent that communicates with product agents and with its own service agent.
 **/
public class EquipletAgent extends Agent implements BlackboardSubscriber {
	/**
	 * @var long serialVersionUID
	 *      The serial version UID.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var AID serviceAgent
	 *      AID of the serviceAgent connected to this EquipletAgent.
	 */
	private AID serviceAgent;

	/**
	 * @var String collectiveDbIp
	 *      IP of the collective database.
	 */
	private String collectiveDbIp = Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp");

	/**
	 * @var int collectiveDbPort
	 *      Port number of the collective database.
	 */
	private int collectiveDbPort = Integer.parseInt(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbPort"));

	/**
	 * @var String collectiveDbName
	 *      Name of the collective database.
	 */
	private String collectiveDbName = Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbName");

	/**
	 * @var String equipletDirectoryName
	 *      Name of the collection containing the equipletDirectory.
	 */
	private String equipletDirectoryName = Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletDirectoryName");

	/**
	 * @var String timeDataName
	 *      Name of the collection containing the timeData.
	 */
	private String timeDataName = Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "timeDataCollectionName");

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

	private ArrayList<Behaviour> behaviours;

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
	 * 
	 */
	private int equipletId;
	/**
	 * @var String equipletDbIp
	 *      IP of the equiplet database.
	 */
	private String equipletDbIp;

	/**
	 * @var int equipletDbPort
	 *      Port number of the equiplet database.
	 */
	private int equipletDbPort;

	/**
	 * @var String equipletDbName
	 *      Name of the equiplet database.
	 */
	private String equipletDbName;

	/**
	 * @var String productStepsName
	 *      Name of the collection containing the productSteps.
	 */
	private String productStepsName;

	
	private static long systemStart = System.currentTimeMillis();

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
		try {
			equipletDbIp = Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbIp", getAID().getLocalName());
			equipletDbPort = Integer.parseInt(Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbPort", getAID().getLocalName()));
		 	equipletDbName = Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "DbName", getAID().getLocalName());
		 	productStepsName = Configuration.getProperty(ConfigurationFiles.EQUIPLET_DB_PROPERTIES, "ProductStepsBlackBoardName", getAID().getLocalName());
		 	
			Logger.log(LogLevel.NOTIFICATION, this.getAID().getLocalName() + " spawned as an equiplet agent.");
			
			communicationTable = new HashMap<String, ObjectId>();
			behaviours = new ArrayList<Behaviour>();
			AID logisticsAgent = (AID) getArguments()[0];

			capabilities = new ArrayList<Integer>();
			KnowledgeDBClient client = KnowledgeDBClient.getClient();
			// Register modules
			Row[] steps = client.executeSelectQuery(Queries.POSSIBLE_STEPS_PER_EQUIPLET, getAID().getLocalName());
			for(Row step : steps) {
				capabilities.add((int) step.get("id"));
			}
			Logger.log(LogLevel.DEBUG, "%s %s%n", capabilities, equipletDbName);

			dbData = new DbData(equipletDbIp, equipletDbPort, equipletDbName);

			Row[] equipletEntrys = client.executeSelectQuery(Queries.SELECT_EQUIPLET_ID, getLocalName());
			equipletId = (int) equipletEntrys[0].get("id");

			Object[] arguments = new Object[] {
					dbData, getAID(), logisticsAgent
			};
			AgentController serviceAgentCnt =
					getContainerController().createNewAgent(getLocalName() + "-serviceAgent",
							"agents.service_agent.ServiceAgent", arguments);
			serviceAgentCnt.start();
			serviceAgent = new AID(serviceAgentCnt.getName(), AID.ISGUID);

			// makes connection with the equiplet blackboard.
			productStepBBClient = new BlackboardClient(equipletDbIp, equipletDbPort);
			productStepBBClient.setDatabase(equipletDbName);
			productStepBBClient.setCollection(productStepsName);

			// subscribes on changes of the status field on the equiplet blackboard.
			statusSubscription = new FieldUpdateSubscription("status", this);
			statusSubscription.addOperation(MongoUpdateLogOperation.SET);
			productStepBBClient.subscribe(statusSubscription);
			productStepBBClient.removeDocuments(new BasicDBObject());

			stateBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			stateBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			stateBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletStateCollectionName"));

			modeUpdateSubscription = new FieldUpdateSubscription("mode", this);
			modeUpdateSubscription.addOperation(MongoUpdateLogOperation.SET);
			stateBBClient.subscribe(modeUpdateSubscription);

			desiredStateBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			desiredStateBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "stateBlackBoardName"));
			desiredStateBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletCommandCollectionName"));

			// makes connection with the collective blackboard.
			collectiveBBClient = new BlackboardClient(collectiveDbIp, collectiveDbPort);
			collectiveBBClient.setDatabase(collectiveDbName);
			collectiveBBClient.setCollection(timeDataName);

			// gets the timedata for synchronizing from the collective blackboard.
			BasicDBObject timeData = (BasicDBObject) collectiveBBClient.findDocuments(new BasicDBObject()).get(0);

			// initiates the timer to the next product step.
			timer = new NextProductStepTimer(timeData.getLong("firstTimeSlot"), timeData.getInt("timeSlotLength"), this);

			collectiveBBClient.setCollection(equipletDirectoryName);
		} catch(GeneralMongoException | InvalidDBNamespaceException | UnknownHostException | StaleProxyException
				| KnowledgeException | KeyNotFoundException  e) {
			Logger.log(LogLevel.CRITICAL, "Could not spawn Equiplet", e);
			//delete this agent and all opened blackboards / started agents
			doDelete();
		} catch(Exception e){
			e.printStackTrace();
			Logger.log(LogLevel.ERROR, e);
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
			Logger.log(LogLevel.ERROR, "Could not clear blackboards for " + this.getAID().getLocalName(), e);
		}

		ACLMessage deadMessage = new ACLMessage(ACLMessage.FAILURE);
		deadMessage.addReceiver(serviceAgent);
		deadMessage.setOntology("EquipletAgentDied");
		send(deadMessage);
	}

	public void cancelProductStep(ObjectId productStepId, String reason) {
		try {
			// TODO cancel all behaviours started specific for this productStep

			productStepBBClient.updateDocuments(
					new BasicDBObject("_id", productStepId),
					new BasicDBObject("$set", new BasicDBObject("status", StepStatusCode.ABORTED.name()).append(
							"statusData", new BasicDBObject("reason", reason))));
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			e.printStackTrace();
		}
	}

	/**
	 * onMessage function for the equipletAgent. Listens to updates of the blackboard clients and handles them.
	 */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			switch(entry.getNamespace().split("\\.")[1]) {
				case "ProductStepsBlackBoard":
					// Get the productstep.
					ObjectId productStepId = entry.getTargetObjectId();
					ProductStep productStep = new ProductStep((BasicDBObject) productStepBBClient.findDocumentById(productStepId));

					// Gets the conversationId
					String conversationId = getConversationId(productStepId);

					// Create the responseMessage
					ACLMessage responseMessage = new ACLMessage(ACLMessage.INFORM);
					responseMessage.addReceiver(productStep.getProductAgentId());
					responseMessage.setConversationId(conversationId);

					Logger.log(LogLevel.INFORMATION, "Equiplet agent - status update: " + productStep.getStatus().toString());
					switch(productStep.getStatus()) {
					// Depending on the changed status fills in the responseMessage and sends it to the product agent.
						case PLANNED:
							try {
								// If the start time of the newly planned productStep is earlier then the next used time
								// slot make it the next used timeslot.
								ScheduleData scheduleData = productStep.getScheduleData();
//								if(timer.getNextUsedTimeSlot() == 0
//										|| scheduleData.getStartTime() < timer.getNextUsedTimeSlot()) {
//									timer.setNextUsedTimeSlot(scheduleData.getStartTime());
//									nextProductStep = productStep.getId();
//								}
								if(timer.getNextUsedTimeSlot() == -1 || scheduleData.getStartTime() < timer.getNextUsedTimeSlot()){
									timer.rescheduleTimer();
								}

								responseMessage.setOntology("Planned");
								responseMessage.setPerformative(ACLMessage.CONFIRM);
								responseMessage.setContentObject(scheduleData.getStartTime());


							} catch(IOException e) {
								responseMessage.setPerformative(ACLMessage.DISCONFIRM);
								responseMessage.setContent("An error occured in the planning/please reschedule");
								Logger.log(LogLevel.ERROR, "Could not serialize scheduledata-starttime", e);
							}
							break;
						case FAILED:
						case SUSPENDED_OR_WARNING:
							setDesiredEquipletState(EquipletState.STANDBY);
							removeCommunicationRelation(productStepId);
							//$FALL-THROUGH$
						case IN_PROGRESS:
						case WAITING:
							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							break;
						case DONE:
							setDesiredEquipletState(EquipletState.STANDBY);
							removeCommunicationRelation(productStepId);

							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							productStep.setStatus(StepStatusCode.DONE);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							productStepBBClient.removeDocuments(new BasicDBObject("_id", productStep.getId()));
							break;
						case DELETED:
							setDesiredEquipletState(EquipletState.STANDBY);
							removeCommunicationRelation(productStepId);

							responseMessage.setOntology("StatusUpdate");
							responseMessage.setPerformative(ACLMessage.CONFIRM);
							responseMessage.setContentObject(productStep.toBasicDBObject());
							productStepBBClient.removeDocuments(new BasicDBObject("_id", productStep.getId()));
							break;
						default:
							break;
					}
					Logger.log(LogLevel.DEBUG, "Equiplet agent - sending message %s%n",
							ACLMessage.getPerformative(responseMessage.getPerformative()));
					send(responseMessage);
					break;
				case "equipletState":
					EquipletStateEntry stateEntry =
							new EquipletStateEntry((BasicDBObject) stateBBClient.findDocumentById(entry
									.getTargetObjectId()));
					Logger.log(LogLevel.DEBUG, "Equiplet agent - mode changed to %s%n", stateEntry.getEquipletMode());
					EquipletMode mode = stateEntry.getEquipletMode();
					switch(mode) {
					// TODO handle error stuff
						case NORMAL:
						case STEP:
						case SERVICE:
							break;
						case ERROR:
						case CRITICAL_ERROR:
						case EMERGENCY_STOP:
						case LOCK:
							break;
						default:
							break;
					}
					break;
				default:
					Logger.log(LogLevel.WARNING, "Equiplet agent - onMessage Unknown database");
					break;
			}
		} catch(GeneralMongoException | InvalidDBNamespaceException | IOException e) {
			// TODO handle error
			Logger.log(LogLevel.ERROR, e);
		}
	}

	public EquipletStateEntry getEquipletStateEntry() throws InvalidDBNamespaceException, GeneralMongoException {
		List<DBObject> equipletStates = stateBBClient.findDocuments(new BasicDBObject("id", equipletId));
		
		if(equipletStates.size() == 0)
			return null;
		
		return new EquipletStateEntry((BasicDBObject) equipletStates.get(0));
	}

	public void setDesiredEquipletState(EquipletState state) throws InvalidDBNamespaceException, GeneralMongoException {
		// TODO when the equipletCommand blackboard has been updated to have a equipletId like field this search query
		// should be adapted.

		// desiredStateBBClient.updateDocuments(new BasicDBObject("id", equipletId), new BasicDBObject("$set",
		// new BasicDBObject("desiredState", state.getValue())));
		desiredStateBBClient.updateDocuments(new BasicDBObject(), new BasicDBObject("$set", new BasicDBObject(
				"desiredState", state.getValue())));
		// desiredStateBBClient.updateDocuments(new BasicDBObject("name", getLocalName()), new BasicDBObject("$set",
		// new BasicDBObject("desiredState", state.getValue())));
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
	 * @param conversationId the conversationId in the new relation.
	 * @param objectId the objectId in the new relation.
	 */
	public void addCommunicationRelation(String conversationId, ObjectId objectId) {
		communicationTable.put(conversationId, objectId);
	}

	/**
	 * Getter for getting the objectId by a conversationId.
	 * 
	 * @param conversationId the conversationId of which the related objectId is needed.
	 * @return ObjectId for the given conversationId.
	 */
	public ObjectId getRelatedObjectId(String conversationId) {
		return communicationTable.get(conversationId);
	}

	/**
	 * Getter for getting the related conversationId for the given ObjectId.
	 * 
	 * @param productStepId the ObjectId for which the related conversationId is needed.
	 * @return the related conversationId or null if the relation does not exist.
	 */
	public String getConversationId(ObjectId productStepId) {
		for(Entry<String, ObjectId> tableEntry : communicationTable.entrySet()) {
			if(tableEntry.getValue().equals(productStepId)) {
				return tableEntry.getKey();
			}
		}
		return null;
	}

	public void removeCommunicationRelation(ObjectId productStepId) {
		String conversationId = null;
		for(Entry<String, ObjectId> tableEntry : communicationTable.entrySet()) {
			if(tableEntry.getValue().equals(productStepId)) {
				conversationId = tableEntry.getKey();
				break;
			}
		}
		communicationTable.remove(conversationId);
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
	
	public static long getCurrentTimeSlot(){
//		return (System.currentTimeMillis() - systemStart)/50;
		return (System.currentTimeMillis())/50;
	}
}



