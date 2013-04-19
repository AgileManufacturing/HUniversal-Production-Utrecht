package serviceAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import java.io.IOException;

import newDataClasses.ScheduleData;
import nl.hu.client.GeneralMongoException;
import nl.hu.client.InvalidDBNamespaceException;

import org.bson.types.ObjectId;

import com.google.gson.Gson;
import com.mongodb.BasicDBObject;

public class AnswerBehaviour extends CyclicBehaviour {
	private static final long serialVersionUID = 1L;

	public AnswerBehaviour(Agent agent) {
		super(agent);
	}

	@Override
	public void action() {
		ServiceAgent agent = (ServiceAgent) getAgent();
		ACLMessage reply, message = agent.receive();
		if (message != null) {
			reply = message.createReply();
			ObjectId content = null;
			BasicDBObject productionStep = null;
			try {
				content = (ObjectId) message.getContentObject();
				BasicDBObject query = new BasicDBObject("_id", content);
				productionStep = (BasicDBObject) agent.getProductionStepBBClient().findDocuments(query).get(0);
			} catch (UnreadableException | InvalidDBNamespaceException | GeneralMongoException e) {
				e.printStackTrace();
				agent.doDelete();
			}
			if (content != null) {
				switch (message.getOntology()) {
				case "CanDoProductionStep":
					boolean isAble = agent.getStepTypes().containsKey(
							((Integer) productionStep.get("type")).longValue());
					reply.setContent("" + isAble);
					try {
						reply.setContentObject(content);
					} catch (IOException e) {
						e.printStackTrace();
						agent.doDelete();
					}
					reply.setOntology("canDoProductionStepResponse");

					if (isAble) {
						 productionStep.removeField("productAgentId");
						 productionStep.removeField("parameters");
						 productionStep.removeField("inputParts");
						 productionStep.removeField("outputParts");
//						 productionStep.removeField("status");
//						 productionStep.removeField("scheduleData");

						StringBuilder strBuilder = new StringBuilder(
								"Production step:\n");
						agent.printDBObjectPretty(productionStep, "    ",
								"    ", strBuilder);
						System.out.format("%s can do step %s%n",
								agent.getLocalName(), strBuilder);
					} else {
						 productionStep.removeField("_id");
//						 productionStep.removeField("status");
//						 productionStep.removeField("scheduleData");

						StringBuilder strBuilder = new StringBuilder(
								"Production step:\n");
						agent.printDBObjectPretty(productionStep, "    ",
								"    ", strBuilder);
						System.out.format("%s cannot do step %s%n",
								agent.getLocalName(), strBuilder);
					}
					break;
				case "GetProductionStepDuration":
					int duration = 0;
					for (String service : agent.getStepTypes().get(
							((Integer) (productionStep.get("type")))
									.longValue())) {
						duration += agent.getServices().get(service);
					}

					ScheduleData scheduleData = new ScheduleData();
					scheduleData.setDuration(duration);

					productionStep.put("scheduleData", scheduleData);
					try {
						BasicDBObject query = new BasicDBObject("_id", productionStep.get("id"));
						BasicDBObject update = new BasicDBObject("$set", scheduleData.toBasicDBObject());
						agent.getProductionStepBBClient().updateDocuments(query, update);
					} catch (InvalidDBNamespaceException | GeneralMongoException e) {
						e.printStackTrace();
						agent.doDelete();
					}

					reply.setContent("Evaluation Complete");
					reply.setOntology("ProductionDurationResponse");

					// System.out.format("%s will do %d timeslots about this step%n",
					// getLocalName(), duration);
					break;
				case "PlanStepWithLogistics":
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
				System.out.format("%s sending %s:%s%n", agent.getLocalName(),
						reply.getOntology(), reply.getContentObject());
			} catch (UnreadableException e) {
				System.out.format("%s sending %s:%s%n", agent.getLocalName(),
						reply.getOntology(), reply.getContent());
			}
			agent.send(reply);
			System.out.format("sent reply to %s%n", ((AID) reply
					.getAllReceiver().next()).getLocalName());
		}
		block();
	}
}
