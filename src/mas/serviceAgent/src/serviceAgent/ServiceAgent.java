package serviceAgent;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.UnreadableException;

import com.google.gson.Gson;
import com.mongodb.*;
import org.bson.types.*;
import nl.hu.client.*;

//TODO add registering with BlackBoard agent for changing productionstep status to WAITING

public class ServiceAgent extends Agent implements BlackboardSubscriber {
	private static final long serialVersionUID = 1L;
	
	private BlackboardClient productionStepBBClient, serviceStepBBClient;
	private Hashtable<String, Long> services;
	private Hashtable<Long, String[]> stepTypes;

    public void setup() {
    	//TODO fill in host, database and collection
    	productionStepBBClient = new BlackboardClient("");
    	serviceStepBBClient = new BlackboardClient("");
    	try {
			productionStepBBClient.setDatabase("CollectionDb");
			productionStepBBClient.setCollection("ProductionStepCollection");
			productionStepBBClient.subscribe(new BlackboardSubscription(MongoOperation.INSERT, this)); //need react on new production steps
			productionStepBBClient.subscribe(new BlackboardSubscription(MongoOperation.UPDATE, this)); //need to react on state changes of production steps to WAITING

			serviceStepBBClient.setDatabase("CollectionDb");
			serviceStepBBClient.setCollection("ServiceStepCollection");
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
    	stepTypes.put(1l, new String[] {"Pick", "Place"});			//Pick&Place
    	stepTypes.put(2l, new String[] {"Glue", "Pick", "Place"});	//Attack
    	stepTypes.put(3l, new String[] {"Drill", "Pick", "Place"});	//Screw
    	
    	addBehaviour(new AnswerBehaviour(this));
    }
    
    public void takeDown() {
    	
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
				List<DBObject> productionStep = null;
				try {
					content = (ObjectId) message.getContentObject();
				} catch (UnreadableException e) {
					e.printStackTrace();
				}
				try {
					productionStep = productionStepBBClient.findDocuments("_id:" + content);
				} catch (InvalidJSONException | InvalidDBNamespaceException e) {
					e.printStackTrace();
				}
				if(content != null) {
					switch(message.getOntology()) {
						case "canDoProductionStep":
							//TODO get step data using content
							//extract stepType
							//is this stepType present in stepTypes?
							//send answer
							
							boolean isAble = stepTypes.containsKey(productionStep.get(0).get("type"));
							reply.setContent("" + isAble);
							reply.setOntology("canDoProductionResponse");
							
							if(isAble)
								System.out.println("Can do step " + content);
							else
								System.out.println("Cannot do step " + content);
							break;
						case "getProductionDuration":
							//TODO get step data using content
							//extract stepType
							//add all durations of those services of this stepTypes
							//send answer
							
							
							int duration = 0;
							
							for(String service : stepTypes.get(productionStep.get(0).get("type"))) {
								duration += services.get(service);
							}
							
							reply.setContent("" + duration);
							reply.setOntology("ProductionDurationResponse");
							
							System.out.println("Step takes " + duration + "timeslots");
							break;
						case "scheduleStepWithLogistics":
							//TODO add behaviour handling scheduling with logistics agent
							
							
//							reply.setContent("" + true);
//							reply.setOntology("DoneScheduling");
					    	addBehaviour(new PlanServiceBehaviour(getAgent(), content));
							
							System.out.println("Scheduled service succesfully");
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
