package MAS.grid_server;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeSet;

import org.json.JSONException;

import MAS.equiplet.Job;
import MAS.util.Parser;

public class ScheduleDispatcher extends Agent {

	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	Map<AID, TreeSet<Job>> schedules;
	
	public void setup() {
		schedules = new HashMap<AID, TreeSet<Job>>();
	}
	
	class DispatcherListenerBehaviour extends Behaviour {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		@Override
		public void action() {
			ACLMessage msg = blockingReceive();
			switch (msg.getPerformative()) { 
			case ACLMessage.SUBSCRIBE:
				handleSubcribeRequest(msg);
				break;
			case ACLMessage.REQUEST:
				break;
			default:
				break;
			}
		}

		@Override
		public boolean done() {
			return false;
		}
		
	}

	protected void handleSubcribeRequest(ACLMessage message) {
		schedules.put(message.getSender(), new TreeSet<Job>());
		
		try {
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			send(reply);
		} catch (JSONException e) {
			System.err.println("Dispatcher: failed to send subscribe confirmation to "+ message.getSender().getLocalName() + " - "+ e.getMessage());
		}
	}
}
