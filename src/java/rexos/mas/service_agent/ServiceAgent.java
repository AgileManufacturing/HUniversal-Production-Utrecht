package rexos.mas.service_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.net.UnknownHostException;
import java.util.HashMap;

import org.bson.types.ObjectId;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.BlackboardSubscriber;
import rexos.libraries.blackboard_client.FieldUpdateSubscription;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.MongoOperation;
import rexos.libraries.blackboard_client.OplogEntry;
import rexos.libraries.log.Logger;
import rexos.mas.data.DbData;
import rexos.mas.equiplet_agent.ProductStepMessage;
import rexos.mas.equiplet_agent.StepStatusCode;
import rexos.mas.service_agent.behaviours.CanDoProductStep;
import rexos.mas.service_agent.behaviours.GetProductStepDuration;
import rexos.mas.service_agent.behaviours.InitialisationFinished;
import rexos.mas.service_agent.behaviours.ScheduleStep;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;
import com.mongodb.DBObject;

/**
 * This agent manages services and oversees generation and scheduling of
 * serviceSteps.
 * 
 * @author Peter
 * 
 */
public class ServiceAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;

	private BlackboardClient productStepBBClient, serviceStepBBClient;
	private FieldUpdateSubscription statusSubscription;
	private DbData dbData;
	private AID equipletAgentAID, hardwareAgentAID, logisticsAID;
	private ServiceFactory serviceFactory;
	private HashMap<String, Service> convIdServiceMapping;

	/* (non-Javadoc)
	 * @see jade.core.Agent#setup() */
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

	/* (non-Javadoc)
	 * @see jade.core.Agent#takeDown() */
	@Override
	public void takeDown() {
		productStepBBClient.unsubscribe(statusSubscription);
		serviceStepBBClient.unsubscribe(statusSubscription);
		try {
			// serviceStepBBClient.removeDocuments(new BasicDBObject());
			Logger.log("ServiceAgent takedown");

			DBObject update =
					BasicDBObjectBuilder.start("status", StepStatusCode.FAILED.name()).push("statusData")
							.add("source", "service agent").add("reason", "died").pop().get();
			productStepBBClient.updateDocuments(new BasicDBObject(), new BasicDBObject("$set", update));

			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(equipletAgentAID);
			message.addReceiver(hardwareAgentAID);
			message.setOntology("ServiceAgentDied");
			send(message);
		} catch(InvalidDBNamespaceException | GeneralMongoException e) {
			Logger.log(e);
		}
	}

	public void MapConvIdWithService(String conversationId, Service service) {
		convIdServiceMapping.put(conversationId, service);
	}

	public Service GetServiceForConvId(String conversationId) {
		return convIdServiceMapping.get(conversationId);
	}

	public void RemoveConvIdServiceMapping(String conversationId) {
		convIdServiceMapping.remove(conversationId);
	}

	/**
	 * @param obj
	 * @param prefix
	 * @param total_prefix
	 * @param result
	 */
	public void printDBObjectPretty(DBObject obj, String prefix, String total_prefix, StringBuilder result) {
		Object value;
		for(String key : obj.keySet()) {
			value = obj.get(key);
			if(value instanceof DBObject) {
				result.append(total_prefix + key + ":\n");
				printDBObjectPretty((DBObject) value, prefix, prefix + total_prefix, result);
			} else if(value == null) {
				result.append(total_prefix + key + ": " + value + "\n");
			} else {
				result.append(total_prefix + key + ": " + value + " (" + value.getClass().getSimpleName() + ")\n");
			}
		}
	}

	/* (non-Javadoc)
	 * @see
	 * rexos.libraries.blackboard_client.BlackboardSubscriber#onMessage(rexos
	 * .libraries.blackboard_client.MongoOperation,
	 * rexos.libraries.blackboard_client.OplogEntry) */
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			switch(entry.getNamespace().split("\\.")[1]) {
				case "ProductStepsBlackBoard":
					ProductStepMessage productionStep =
							new ProductStepMessage((BasicDBObject) productStepBBClient.findDocumentById(entry
									.getTargetObjectId()));
					switch(operation) {
						case UPDATE:
							StepStatusCode status = productionStep.getStatus();
							if(status == StepStatusCode.WAITING) {
								serviceStepBBClient.updateDocuments(
										new BasicDBObject("productStepId", entry.getTargetObjectId()),
										new BasicDBObject("$set", new BasicDBObject("status", status)));
							}
							break;
						case DELETE:
							serviceStepBBClient.removeDocuments(new BasicDBObject("productStepId", entry
									.getTargetObjectId()));
							break;
						default:
							break;
					}
					break;
				case "ServiceStepsBlackBoard":
					BasicDBObject serviceStep =
							(BasicDBObject) serviceStepBBClient.findDocumentById(entry.getTargetObjectId());
					ObjectId productStepId = (ObjectId) serviceStep.get("productStepId");
					switch(operation) {
						case UPDATE:
							BasicDBObject update = new BasicDBObject("status", serviceStep.get("status"));
							update.put("statusData", serviceStep.get("statusData"));
							productStepBBClient.updateDocuments(new BasicDBObject("_id", productStepId),
									new BasicDBObject("$set", update));
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
	 * @return the dbData
	 */
	public DbData getDbData() {
		return dbData;
	}

	/**
	 * @return the equipletAgentAID
	 */
	public AID getEquipletAgentAID() {
		return equipletAgentAID;
	}

	/**
	 * @return the logisticsAID
	 */
	public AID getLogisticsAID() {
		return logisticsAID;
	}

	/**
	 * @return the hardwareAgentAID
	 */
	public AID getHardwareAgentAID() {
		return hardwareAgentAID;
	}

	/**
	 * @return the productStepBBClient
	 */
	public BlackboardClient getProductStepBBClient() {
		return productStepBBClient;
	}

	/**
	 * @return the serviceStepBBClient
	 */
	public BlackboardClient getServiceStepBBClient() {
		return serviceStepBBClient;
	}
}
