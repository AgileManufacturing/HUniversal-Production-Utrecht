/**
 * 
 */
package ProductAgent;

import jade.core.Agent;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * @author Alexander
 * Receiver class based on the ReceiverBehaviour.
 * instead of polling .done(), this class will return when either the timeout fires or a msg is received.
 */
public class ReceiveBehaviour extends SimpleBehaviour {

	private MessageTemplate template;
	private long timeOut, wakeupTime;
	private boolean finished;

	private ACLMessage msg;

	public ACLMessage getMessage() {
		return msg;
	}

	public ReceiveBehaviour(Agent a, int millis, MessageTemplate mt) {
		super(a);
		timeOut = millis;
		template = mt;
	}

	public void onStart() {
		wakeupTime = (timeOut < 0 ? Long.MAX_VALUE : System
				.currentTimeMillis() + timeOut);
	}

	public boolean done() {
		return finished;
	}

	public void action() {
		if (template == null)
			msg = myAgent.receive();
		else
			msg = myAgent.receive(template);

		if (msg != null) {
			finished = true;
			handle(msg);
			return;
		}
		long dt = wakeupTime - System.currentTimeMillis();
		if (dt > 0)
			block(dt);
		else {
			finished = true;
			handle(msg);
		}
	}
	
	 /*
	  * This function will be called in the sub_class e.g.
	  * new ReceiveBehaviour(myAgent, 10000, template) {
				public void handle(ACLMessage msg) {
					if(msg == null) timeout expired
					
					handle msg stuff.
				}
		}
	  */
	public void handle(ACLMessage m)
	{}

	public void reset() {
		msg = null;
		finished = false;
		super.reset();
	}

	public void reset(int dt) {
		timeOut = dt;
		reset();
	}

}