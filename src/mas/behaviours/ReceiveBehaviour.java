package behaviours;

import jade.core.Agent;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
class ReceiveBehaviour extends SimpleBehaviour {

	private MessageTemplate template;
	private long timeOut, wakeupTime;
	private boolean finished;

	private ACLMessage msg;

	/**
	 * Gets the message.
	 *
	 * @return the message
	 */
	public ACLMessage getMessage() {
		return msg;
	}

	/**
	 * Instantiates a new receive behaviour.
	 *
	 * @param a the a
	 * @param millis the millis
	 * @param mt the mt
	 */
	public ReceiveBehaviour(Agent a, int millis, MessageTemplate mt) {
		super(a);
		timeOut = millis;
		template = mt;
	}

	public void onStart() {
		wakeupTime = (timeOut < 0 ? Long.MAX_VALUE : System.currentTimeMillis()
				+ timeOut);
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

	/**
	 * Handle.
	 *
	 * @param m the m
	 */
	public void handle(ACLMessage m) { /* can be redefined in sub_class */
	}

	public void reset() {
		msg = null;
		finished = false;
		super.reset();
	}

	/**
	 * Reset.
	 *
	 * @param dt the dt
	 */
	public void reset(int dt) {
		timeOut = dt;
		reset();
	}

}