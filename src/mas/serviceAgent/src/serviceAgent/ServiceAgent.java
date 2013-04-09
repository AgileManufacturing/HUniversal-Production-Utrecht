package serviceAgent;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Hashtable;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import com.google.gson.Gson;
import com.google.gson.JsonSyntaxException;
import com.mongodb.*;

import equipletAgent.*;
import org.bson.types.*;

import newDataClasses.ScheduleData;
import nl.hu.client.*;

//TODO add registering with BlackBoard agent for changing productionstep status to WAITING

public class ServiceAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;

	private BlackboardClient productionStepBBClient, serviceStepBBClient;
	private Hashtable<String, Long> services;
	private Hashtable<Long, String[]> stepTypes;
	private DbData dbData;

	public void setup() {
		System.out.println("I spawned as a service agent.");

		// TODO fill in host, database and collection
		Object[] args = getArguments();
		if (args != null && args.length > 0) {
			dbData = (DbData) args[0];
		}

		try {
			productionStepBBClient = new BlackboardClient(dbData.ip);
			serviceStepBBClient = new BlackboardClient(dbData.ip);

			productionStepBBClient.setDatabase(dbData.name);
			productionStepBBClient.setCollection("ProductStepsBlackBoard");
			productionStepBBClient.subscribe(new BasicOperationSubscription(
					MongoOperation.INSERT, this)); // need react on new
													// production steps
			productionStepBBClient.subscribe(new BasicOperationSubscription(
					MongoOperation.UPDATE, this)); // need to react on state
													// changes of production
													// steps to WAITING
			productionStepBBClient.subscribe(new BasicOperationSubscription(
					MongoOperation.DELETE, this)); // Need to react on deletion
													// of steps which means that
													// they are cancelled.

			serviceStepBBClient.setDatabase(dbData.name);
			serviceStepBBClient.setCollection("ServiceStepsBlackBoard");
			serviceStepBBClient.subscribe(new BasicOperationSubscription(
					MongoOperation.UPDATE, this)); // need to react on state
		} catch (UnknownHostException | GeneralMongoException
				| InvalidDBNamespaceException e) {
			e.printStackTrace();
			doDelete();
		}

		services = new Hashtable<String, Long>();
		services.put("Drill", 15l);
		services.put("Glue", 20l);
		services.put("Pick", 5l);
		services.put("Place", 5l);

		stepTypes = new Hashtable<Long, String[]>();
		stepTypes.put(0l, new String[] { "Pick", "Place" }); // Pick&Place
		stepTypes.put(1l, new String[] { "Glue", "Pick", "Place" }); // Attack
		stepTypes.put(2l, new String[] { "Drill", "Pick", "Place" }); // Screw
		stepTypes.put(3l, new String[] { "Drill", "Pick", "Place" }); // Screw

		addBehaviour(new AnswerBehaviour(this));
	}

	public void takeDown() {
		// TODO implement graceful death
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

	private class AnswerBehaviour extends CyclicBehaviour {
		private static final long serialVersionUID = 1L;

		public AnswerBehaviour(Agent agent) {
			super(agent);
		}

		@Override
		public void action() {
			ACLMessage reply, message = receive();
			if (message != null) {
				reply = message.createReply();
				ObjectId content = null;
				DBObject productionStep = null;
				try {
					content = (ObjectId) message.getContentObject();
					BasicDBObject query = new BasicDBObject();
					query.put("_id", content);
					productionStep = productionStepBBClient
							.findDocuments(query).get(0);
				} catch (UnreadableException | InvalidDBNamespaceException
						| GeneralMongoException e) {
					e.printStackTrace();
					doDelete();
				}
				if (content != null) {
					switch (message.getOntology()) {
					case "canDoProductionStep":
						boolean isAble = stepTypes
								.containsKey(((Integer) productionStep
										.get("type")).longValue());
						reply.setContent("" + isAble);
						try {
							reply.setContentObject(content);
						} catch (IOException e) {
							e.printStackTrace();
							doDelete();
						}
						reply.setOntology("canDoProductionStepResponse");

						if (isAble) {
							productionStep.removeField("productAgentId");
							productionStep.removeField("parameters");
							productionStep.removeField("inputParts");
							productionStep.removeField("outputParts");
							productionStep.removeField("status");
							productionStep.removeField("scheduleData");

							StringBuilder strBuilder = new StringBuilder(
									"Production step:\n");
							printDBObjectPretty(productionStep, "    ", "    ",
									strBuilder);
							// System.out.format("%s can do step %s%n",
							// getLocalName(), strBuilder);
						} else {
							productionStep.removeField("_id");
							productionStep.removeField("status");
							productionStep.removeField("scheduleData");

							StringBuilder strBuilder = new StringBuilder(
									"Production step:\n");
							printDBObjectPretty(productionStep, "    ", "    ",
									strBuilder);
							// System.out.format("%s cannot do step %s%n",
							// getLocalName(), strBuilder);
						}
						break;
					case "getProductionStepDuration":
						int duration = 0;
						for (String service : stepTypes
								.get(((Integer) (productionStep.get("type")))
										.longValue())) {
							duration += services.get(service);
						}

						ScheduleData scheduleData = new ScheduleData();
						scheduleData.setDuration(duration);

						Gson gson = new Gson();
						productionStep.put("scheduleData", scheduleData);
						try {
							productionStepBBClient
									.updateDocuments(
											new BasicDBObject("_id",
													productionStep.get("_id")),
											new BasicDBObject(
													"$set",
													new BasicDBObject(
															"scheduleData",
															BlackboardClient
																	.parseJSONWithCheckException(gson
																			.toJson(scheduleData)))));
						} catch (InvalidDBNamespaceException
								| GeneralMongoException | InvalidJSONException e) {
							e.printStackTrace();
							doDelete();
						}

						reply.setContent("Evaluation Complete");
						reply.setOntology("ProductionDurationResponse");

						// System.out.format("%s will do %d timeslots about this step%n",
						// getLocalName(), duration);
						break;
					case "planStepWithLogistics":
						// TODO add behaviour handling scheduling with logistics
						// agent

						// reply.setContent("" + true);
						// reply.setOntology("DoneScheduling");
						// addBehaviour(new PlanServiceBehaviour(getAgent(),
						// content));
						break;
					default:
						reply.setContent("Unknown ontology");
						System.out.format("Unknown ontology: %s content: %s%n",
								message.getOntology(), content);
						break;
					}
				} else {
					reply.setContent("No content");
				}

				try {
					System.out.format("%s sending %s:%s%n", getLocalName(),
							reply.getOntology(), reply.getContentObject());
				} catch (UnreadableException e) {
					System.out.format("%s sending %s:%s%n", getLocalName(),
							reply.getOntology(), reply.getContent());
				}
				send(reply);
				System.out.format("sent reply to %s%n", ((AID) reply
						.getAllReceiver().next()).getLocalName());
			}
			block();
		}
	}

	private class PlanServiceBehaviour extends OneShotBehaviour {
		private static final long serialVersionUID = 1L;

		private String service;

		public PlanServiceBehaviour(Agent agent, ObjectId stepID) {
			super(agent);
			this.service = service;
		}

		@Override
		public void action() {
			// TODO negotiate with logistics agent to determine when parts can
			// be here
			// send a message to equiplet agent with the answer.

			System.out.format("planning service %s%n", service);
		}
	}

	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
		switch (entry.getNamespace().split(".")[1]) {
		case "ProductStepsBlackBoard":

			switch (operation) {
			case INSERT:
				addBehaviour(new ReportStepDurationBehaviour(this,
						entry.getTargetObjectId()));
				break;
			case UPDATE:
				addBehaviour(new DoServiceBehaviour(this, ));
				break;
			case DELETE:
				break;
			default:
				break;
			}
			break;
		case "ServiceStepsBlackBoard":
			break;
		}
	}

	private class DoServiceBehaviour extends SimpleBehaviour {
		private static final long serialVersionUID = 1L;

		private String service;

		public DoServiceBehaviour(Agent agent, String service) {
			super(agent);
			this.service = service;

			System.out.format("planning service %s%n", service);
		}

		@Override
		public void action() {
			System.out.format("service %s done%n", service);
			// TODO update status step in production step BB
		}

		@Override
		public boolean done() {
			return false;
		}
	}

	private class ReportStepDurationBehaviour extends OneShotBehaviour {
		static final long serialVersionUID = 1L;
		private ObjectId stepId;

		public ReportStepDurationBehaviour(Agent a, ObjectId stepId) {
			super(a);
			this.stepId = stepId;
		}

		@Override
		public void action() {
			BasicDBObject query = new BasicDBObject("_id", stepId);

			BasicDBObject productionStep = null;
			try {
				productionStep = (BasicDBObject) productionStepBBClient
						.findDocuments(query).get(0);
			} catch (InvalidDBNamespaceException | GeneralMongoException e) {
				e.printStackTrace();
				doDelete();
			}

			boolean isAble = stepTypes.containsKey(((Integer) productionStep
					.get("type")).longValue());

			if (isAble) {
				int duration = 0;
				for (String service : stepTypes.get(((Integer) (productionStep
						.get("type"))).longValue())) {
					duration += services.get(service);
				}

				ScheduleData scheduleData = new ScheduleData();
				scheduleData.setDuration(duration);

				Gson gson = new Gson();
				productionStep.put("scheduleData", scheduleData);
				try {
					productionStepBBClient.updateDocuments(
							query,
							new BasicDBObject("$set", new BasicDBObject(
									"scheduleData", BlackboardClient
											.parseJSONWithCheckException(gson
													.toJson(scheduleData)))));
				} catch (InvalidDBNamespaceException | GeneralMongoException
						| InvalidJSONException e) {
					e.printStackTrace();
					doDelete();
				}

				productionStep.removeField("productAgentId");
				productionStep.removeField("parameters");
				productionStep.removeField("inputParts");
				productionStep.removeField("outputParts");
				productionStep.removeField("status");
				productionStep.removeField("scheduleData");

				StringBuilder strBuilder = new StringBuilder(
						"Production step:\n");
				printDBObjectPretty(productionStep, "    ", "    ", strBuilder);
				System.out.format("%s can do step %s%n", getLocalName(),
						strBuilder);
			} else {
				ACLMessage message = new ACLMessage(ACLMessage.DISCONFIRM);
				message.setOntology("canDoProductionStepResponse");
				message.setContent("Not able");
				try {
					message.setContentObject(stepId);
				} catch (IOException e) {
					e.printStackTrace();
					doDelete();
				}
				addBehaviour(new SenderBehaviour(ServiceAgent.this, message));

				productionStep.removeField("_id");
				productionStep.removeField("status");
				productionStep.removeField("scheduleData");

				StringBuilder strBuilder = new StringBuilder(
						"Production step:\n");
				printDBObjectPretty(productionStep, "    ", "    ", strBuilder);
				System.out.format("%s cannot do step %s%n", getLocalName(),
						strBuilder);
			}
		}
	}
}
