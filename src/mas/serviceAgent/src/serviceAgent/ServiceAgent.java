package serviceAgent;

import java.io.IOException;
import java.util.Hashtable;
import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import com.google.gson.Gson;
import com.mongodb.*;
import equipletAgent.*;
import org.bson.types.*;

import nl.hu.client.*;

//TODO add registering with BlackBoard agent for changing productionstep status to WAITING

public class ServiceAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;
	
	private BlackboardClient productionStepBBClient, serviceStepBBClient;
	private Hashtable<String, Long> services;
	private Hashtable<Long, String[]> stepTypes;
	private DbData dbData;

    public void setup() {
    	//TODO fill in host, database and collection
    	Object[] args = getArguments();
    	if (args != null && args.length > 0) {
    		dbData = (DbData)args[0];
		}
    	
    	productionStepBBClient = new BlackboardClient(dbData.ip);
    	serviceStepBBClient = new BlackboardClient(dbData.ip);
    	try {
			productionStepBBClient.setDatabase(dbData.name);
			productionStepBBClient.setCollection("ProductStepsBlackBoard");
			productionStepBBClient.subscribe(new BlackboardSubscription(MongoOperation.INSERT, this)); //need react on new production steps
			productionStepBBClient.subscribe(new BlackboardSubscription(MongoOperation.UPDATE, this)); //need to react on state changes of production steps to WAITING

			serviceStepBBClient.setDatabase(dbData.name);
			serviceStepBBClient.setCollection("ProductionStepBlackBoard");
			serviceStepBBClient.subscribe(new BlackboardSubscription(MongoOperation.UPDATE, this)); //need to react on state changes of service steps
		} catch (Exception e) {
			e.printStackTrace();
		}
    	
    	services = new Hashtable<String, Long>();
    	services.put("Drill", 15l);
    	services.put("Glue", 20l);
    	services.put("Pick", 5l);
    	services.put("Place", 5l);

    	stepTypes = new Hashtable<Long, String[]>();
    	stepTypes.put(0l, new String[] {"Pick", "Place"});			//Pick&Place
    	stepTypes.put(1l, new String[] {"Glue", "Pick", "Place"});	//Attack
    	stepTypes.put(2l, new String[] {"Drill", "Pick", "Place"});	//Screw
    	stepTypes.put(3l, new String[] {"Drill", "Pick", "Place"});	//Screw
    	
    	addBehaviour(new AnswerBehaviour(this));
    }
    
    public void takeDown() {
    	
    }

    public void printDBObjectPretty(DBObject obj, String prefix, String total_prefix, StringBuilder result) {
    	Object value;
		for(String key : obj.keySet()) {
			value = obj.get(key);
			if(value instanceof DBObject) {
				result.append(total_prefix + key + ":\n");
				printDBObjectPretty((DBObject) value, prefix, prefix + total_prefix, result);
			}
			else if(value == null) {
				result.append(total_prefix + key + ": " + value + "\n");
			} else {
				result.append(total_prefix + key + ": " + value + " (" + value.getClass().getSimpleName() + ")\n");
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
			if(message != null) {
				reply = message.createReply();
				ObjectId content = null;
				DBObject productionStep = null;
				try {
					content = (ObjectId) message.getContentObject();
					BasicDBObject query = new BasicDBObject();
					query.put("_id", content);
					productionStep = productionStepBBClient.findDocuments(query).get(0);
				} catch (UnreadableException | InvalidDBNamespaceException e) {
					e.printStackTrace();
				}
				if(content != null) {
					switch(message.getOntology()) {
						case "canDoProductionStep":
							//TODO get step data using content
							//extract stepType
							//is this stepType present in stepTypes?
							//send answer
							
							boolean isAble = stepTypes.containsKey(((Integer)productionStep.get("type")).longValue());
							reply.setContent("" + isAble);
							try {
								reply.setContentObject(content);
							} catch (IOException e1) {
								e1.printStackTrace();
							}
							reply.setOntology("canDoProductionStepResponse");
							
							if(isAble) {
								productionStep.removeField("productAgentId");
								productionStep.removeField("parameters");
								productionStep.removeField("inputParts");
								productionStep.removeField("outputParts");
								productionStep.removeField("status");
								productionStep.removeField("scheduleData");
								System.out.println(getLocalName() + " can do step");
							} else {
								productionStep.removeField("_id");
								productionStep.removeField("status");
								productionStep.removeField("scheduleData");
								System.out.println(getLocalName() + " cannot do step");
							}
							StringBuilder strBuilder = new StringBuilder("Production step:\n");
							printDBObjectPretty(productionStep, "    ", "    ", strBuilder);
							System.out.println(strBuilder);
							break;
						case "getProductionStepDuration":
							//TODO get step data using content
							//extract stepType
							//add all durations of those services of this stepTypes
							//send answer
							
							
							int duration = 0;
							for(String service : stepTypes.get(((Integer)(productionStep.get("type"))).longValue())) {
								duration += services.get(service);
							}

//							ScheduleData scheduleData = (ScheduleData) productionStep.get("scheduleData");
							ScheduleData scheduleData = new ScheduleData();
							scheduleData.setDuration(duration);
							
							Gson gson = new Gson();
							productionStep.put("scheduleData", scheduleData);
							try {
//								System.out.println("{ _id: ObjectId(\"" + productionStep.get("_id") + "\") }");
//								System.out.println("updated " +
//										productionStepBBClient.updateDocuments(
//											"{ _id: ObjectId(\"" + productionStep.get("_id") + "\") }",
//											"{ $set: { \"scheduleData\": " + gson.toJson(scheduleData) + " } }")
//										+ " documents");

								System.out.println("updated " +
										productionStepBBClient.updateDocuments(
											new BasicDBObject("_id", productionStep.get("_id")),
											new BasicDBObject("$set", new BasicDBObject("scheduleData", gson.fromJson(gson.toJson(scheduleData), BasicDBObject.class))))
										+ " documents");
							} catch (InvalidDBNamespaceException e) {
								e.printStackTrace();
							}
							
							reply.setContent("Evaluation Complete");
							reply.setOntology("ProductionDurationResponse");
							
							System.out.println("Step takes " + duration + " timeslots");
							break;
						case "scheduleStepWithLogistics":
							//TODO add behaviour handling scheduling with logistics agent
							
							
//							reply.setContent("" + true);
//							reply.setOntology("DoneScheduling");
//					    	addBehaviour(new PlanServiceBehaviour(getAgent(), content));
							break;
						default:
							reply.setContent("Unknown ontology");
							System.out.println("Unknown ontology: " + message.getOntology() + " content: " + content);
							break;
					}
				} else {
					reply.setContent("No content");
				}
				send(reply);
			}
			block();
		}
    }
    
//    private <T> T JSONtoObject(Object o) {
//    	return new T();
//    }
    
    private class PlanServiceBehaviour extends OneShotBehaviour {
		private static final long serialVersionUID = 1L;
		
		private String service;

		public PlanServiceBehaviour(Agent agent, ObjectId stepID) {
			super(agent);
			this.service = service;
		}

		@Override
		public void action() {
			//TODO negociate with logistics agent to determine when parts can be here
			//send a message to equiplet agent with the answer.
			
			System.out.println("planning service " + service);
		}
    }
    
    private class DoServiceBehaviour extends SimpleBehaviour {
		private static final long serialVersionUID = 1L;
		
		private String service;

		public DoServiceBehaviour(Agent agent, String service) {
			super(agent);
			this.service = service;

			System.out.println("executing service " + service);
		}

		@Override
		public void action() {
			System.out.println("service " + service + " done");
			//TODO update status step in production step BB
		}

		@Override
		public boolean done() {
			return false;
		}
    }
    
	@Override
	public void onMessage(MongoOperation operation, OplogEntry entry) {
	    //TODO implement onMessage
	}
}
