package serviceAgent;

import java.util.Hashtable;
import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;
import com.mongodb.*;
import org.bson.types.*;

//TODO add registering with BlackBoard agent for changing productionstep status to WAITING

public class ServiceAgent extends Agent {
	private static final long serialVersionUID = 1L;
	private Hashtable<String, Long> services;
	private Hashtable<String, String[]> stepTypes;

    public void setup() {
    	services = new Hashtable<String, Long>();
    	services.put("Drill", 15l);
    	services.put("Glue", 20l);
    	services.put("Pick", 5l);
    	services.put("Place", 5l);

    	stepTypes = new Hashtable<String, String[]>();
    	stepTypes.put("Pick&Place", new String[] {"Pick", "Place"});
    	stepTypes.put("Attack", new String[] {"Glue", "Pick", "Place"});
    	stepTypes.put("Screw", new String[] {"Drill", "Pick", "Place"});
    	
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
				ObjectId content = (ObjectId) message.getContentObject();
				if(content != null) {
					switch(message.getOntology()) {
						case "canDoProductionStep":
							//TODO get step data using content
							//lookup what services are needed
							//are those services available?
							//send answer
							
							
							int stepID = Integer.parseInt(content);
							boolean isAble = stepTypes.size() < stepID;
							reply.setContent("" + isAble);
							reply.setOntology("isAble");
							
							if(isAble)
								System.out.println("Can do step " + stepID);
							else
								System.out.println("Cannot do step " + stepID);
							break;
						case "getProductionStepDuration":
							//TODO get step data using content
							//lookup what services are needed
							//add all durations of those services
							//send answer
							
							
							int duration = 0;
							
							for(String service : stepTypes.get(Integer.parseInt(content))) {
								duration += services.get(service);
							}
							
							reply.setContent("" + duration);
							reply.setOntology("Duration");
							
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
    
    public void onMessage(/*TODO params*/) {
    	addBehaviour(new DoServiceBehaviour(this, null/*service uit params*/));
    }
}
