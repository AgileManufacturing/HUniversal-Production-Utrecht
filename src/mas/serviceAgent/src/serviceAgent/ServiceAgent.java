package serviceAgent;

import java.net.UnknownHostException;
import java.util.HashMap;

import serviceAgent.behaviour.CanDoProductStep;
import serviceAgent.behaviour.GetProductDuration;
import jade.core.AID;
import jade.core.Agent;
import com.mongodb.*;
import equipletAgent.*;
import newDataClasses.DbData;
import nl.hu.client.*;

public class ServiceAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;

	private BlackboardClient productionStepBBClient, serviceStepBBClient;
	private FieldUpdateSubscription statusSubscription = new FieldUpdateSubscription(
			"status", this);
	private HashMap<String, Long> services;
	private HashMap<Long, String[]> stepTypes;
	private DbData dbData;
	private AID equipletAgentAID, hardwareAgentAID;

	@Override
	public void setup() {
		System.out.println("I spawned as a service agent.");

		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
			equipletAgentAID = (AID) args[1];
			hardwareAgentAID = (AID) args[2];
		}

		try {
			productionStepBBClient = new BlackboardClient(dbData.ip);
			serviceStepBBClient = new BlackboardClient(dbData.ip);

			productionStepBBClient.setDatabase(dbData.name);
			productionStepBBClient.setCollection("ProductStepsBlackBoard");
			productionStepBBClient.subscribe(statusSubscription); // need to
																	// react on
																	// state
																	// changes
																	// of
																	// production
																	// steps to
																	// WAITING

			serviceStepBBClient.setDatabase(dbData.name);
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");
			serviceStepBBClient.subscribe(statusSubscription); // need to react
																// on status
		} catch (UnknownHostException | GeneralMongoException
				| InvalidDBNamespaceException e) {
			e.printStackTrace();
			doDelete();
		}

		services = new HashMap<>();
		services.put("Drill", 15l);
		services.put("Glue", 20l);
		services.put("Pick", 5l);
		services.put("Place", 5l);

		stepTypes = new HashMap<>();
		stepTypes.put(0l, new String[] { "Pick", "Place" }); // Pick&Place
		stepTypes.put(1l, new String[] { "Glue", "Pick", "Place" }); // Attack
		stepTypes.put(2l, new String[] { "Drill", "Pick", "Place" }); // Screw
		stepTypes.put(3l, new String[] { "Drill", "Pick", "Place" }); // Screw

		// create serviceFactory
		// addBehaviour(new AnswerBehaviour(this));
		addBehaviour(new CanDoProductStep(this, productionStepBBClient));
		addBehaviour(new GetProductDuration(this, productionStepBBClient,
				serviceStepBBClient));

		// receive behaviours from EA
		// add EvaluateProductionStep receiveBehaviour --> conversation with HA
		// add GetStartTime receiveBehaviour --> conversation with LA
		// add ScheduleStep receiveBehaviour
		// add StepDuration receiveBehaviour
		// add StepDuration receiveBehaviour
	}

	@Override
	public void takeDown() {
		productionStepBBClient.unsubscribe(statusSubscription);
		serviceStepBBClient.unsubscribe(statusSubscription);
		// TODO clear serviceStepBB
		// TODO set status of all productionStepBB to ABORTED
	}

	public void printDBObjectPretty(DBObject obj, String prefix,
			String total_prefix, StringBuilder result) {
		Object value;
		for (String key : obj.keySet()) {
			value = obj.get(key);
			if (value instanceof DBObject) {
				result.append(total_prefix + key + ":\n");
				printDBObjectPretty((DBObject) value, prefix, prefix
						+ total_prefix, result);
			} else if (value == null) {
				result.append(total_prefix + key + ": " + value + "\n");
			} else {
				result.append(total_prefix + key + ": " + value + " ("
						+ value.getClass().getSimpleName() + ")\n");
			}
		}
	}

	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		try {
			BasicDBObject productionStep = (BasicDBObject) productionStepBBClient
					.findDocuments(
							new BasicDBObject("_id", entry.getTargetObjectId()))
					.get(0);

			switch (entry.getNamespace().split(".")[1]) {
			case "ProductStepsBlackBoard":

				switch (operation) {
				case INSERT:
					// addBehaviour(new GetProductDuration(this,
					// productionStep));
					break;
				case UPDATE:
					StepStatusCode status = (StepStatusCode) productionStep
							.get("status");
					if (status == StepStatusCode.WAITING) {
						// for (String service : stepTypes.get(productionStep
						// .get("type")));
						// addBehaviour(new DoServiceBehaviour(this, service));
					}
					break;
				case DELETE:
					// TODO remove servicesteps from serviceStepBB
					break;
				// $CASES-OMITTED$
				default:
					break;
				}
				break;
			case "ServiceStepsBlackBoard":
				break;
			default:
				break;
			}
		} catch (InvalidDBNamespaceException | GeneralMongoException e) {
			e.printStackTrace();
			doDelete();
		}
	}

	/**
	 * @return the services
	 */
	public HashMap<String, Long> getServices() {
		return services;
	}

	/**
	 * @return the stepTypes
	 */
	public HashMap<Long, String[]> getStepTypes() {
		return stepTypes;
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
	 * @return the hardwareAgentAID
	 */
	public AID getHardwareAgentAID() {
		return hardwareAgentAID;
	}

	/**
	 * @return the productionStepBBClient
	 */
	public BlackboardClient getProductionStepBBClient() {
		return productionStepBBClient;
	}

	/**
	 * @return the serviceStepBBClient
	 */
	public BlackboardClient getServiceStepBBClient() {
		return serviceStepBBClient;
	}
}
