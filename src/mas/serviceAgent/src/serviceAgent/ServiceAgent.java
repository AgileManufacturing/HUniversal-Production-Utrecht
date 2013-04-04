package serviceAgent;

import java.util.Hashtable;
import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;

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
				String content = message.getContent();
				switch(message.getOntology()) {
					case "canDoProductionStep":
						//TODO check production step BB for step type
						int stepID = Integer.parseInt(content);
						boolean isAble = stepTypes.size() < stepID;
						reply.setContent("" + isAble);
						
						if(isAble)
							System.out.println("Can do step " + stepID);
						else
							System.out.println("Cannot do step " + stepID);
						break;
					case "getProductionStepDuration":
						//TODO split the step in equiplet steps and ask the hardware agents how long these will take
						int duration = 0;
						
						for(String service : stepTypes.get(Integer.parseInt(content))) {
							duration += services.get(service);
						}
						
						reply.setContent("" + duration);
						
						System.out.println("Step takes " + duration + "timeslots");
						break;
					case "scheduleProductionStep":
						reply.setContent("Step scheduled");
				    	addBehaviour(new PlanServiceBehaviour(getAgent(), content));
						
						System.out.println("Scheduled service succesfully");
						break;
					default:
						System.out.println("Unknown ontology: " + message.getOntology() + " content: " + content);
						break;
				}
				send(reply);
			}
			block();
		}
    }
    
    private class PlanServiceBehaviour extends OneShotBehaviour {
		private static final long serialVersionUID = 1L;
		
		private String service;

		public PlanServiceBehaviour(Agent agent, String service) {
			super(agent);
			this.service = service;
		}

		@Override
		public void action() {
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
			//update status step in production step BB
		}

		@Override
		public boolean done() {
			return false;
		}
    }
    
    public void onMessage(/*params*/) {
    	addBehaviour(new DoServiceBehaviour(this, null/*service uit params*/));
    }
}
