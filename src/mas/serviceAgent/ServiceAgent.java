package serviceAgent;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Hashtable;

import nl.hu.client.BlackboardClient;

import com.google.gson.Gson;
import com.mongodb.Mongo;

import equipletAgent.DbData;
import equipletAgent.EquipletDirectoryMessage;
import jade.core.Agent;
import jade.core.behaviours.*;
import jade.lang.acl.ACLMessage;

public class ServiceAgent extends Agent {
	private static final long serialVersionUID = 1L;
	private Hashtable<String, Long> services;

    public void setup() {
    	services = new Hashtable<String, Long>();
    	services.put("Drill", 15l);
    	services.put("Glue", 20l);
    	services.put("Pick", 5l);
    	services.put("Place", 5l);
    	
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
					case "getServiceDuration":
						reply.setContent("" + services.get(content));
						
						System.out.println("Service takes " + reply.getContent() + "timeslots");
						break;
					case "scheduleService":
						reply.setContent("" + true);
				    	addBehaviour(new DoServiceBehaviour(getAgent(), content, services.get(content)));
						
						System.out.println("Scheduled service succesfully");
						break;
					default:
						break;
				}
			}
			block();
		}
    }
    
    private class DoServiceBehaviour extends WakerBehaviour {
		private static final long serialVersionUID = 1L;
		
		private String service;

		public DoServiceBehaviour(Agent agent, String service, long duration) {
			super(agent, duration);
			this.service = service;
		}
		
		public void handleElapsedTimeout() {
			System.out.println("service " + service + " done");
		}
    }
}
