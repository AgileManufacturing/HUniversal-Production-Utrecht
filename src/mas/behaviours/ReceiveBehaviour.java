package behaviours;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

@SuppressWarnings("serial")
abstract class ReceiveBehaviour extends CyclicBehaviour {

	private MessageTemplate template;
	private long timeOut, wakeupTime;

	private ACLMessage msg;

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

	public void action() {
		while((msg = myAgent.receive(template)) != null) {
			handle(msg);
		}
		long dt = wakeupTime - System.currentTimeMillis();
		if (dt > 0)
			block(dt);
		else {
			handle((ACLMessage) null);
		}
	}

	/**
	 * Gets the message.
	 *
	 * @return the message
	 */
	public ACLMessage getMessage() {
		return msg;
	}
	
	public void setTimeOut(long milis) {
		timeOut = milis;
		onStart();
	}
	
	public void clearTimeOut() {
		timeOut = -1;
		onStart();
	}

	public void onStart() {
		wakeupTime = (timeOut < 0 ? Long.MAX_VALUE : System.currentTimeMillis()
				+ timeOut);
	}

	public void reset() {
		super.reset();
		msg = null;
	}

	/**
	 * Reset.
	 *
	 * @param dt the dt
	 */
	public void reset(int dt) {
		reset();
		timeOut = dt;
	}

	/**
	 * Handle.
	 *
	 * @param m the m
	 */
	public abstract void handle(ACLMessage m); /* can be redefined in sub_class */
	
	public abstract class ReceiveOnceBehaviour extends ReceiveBehaviour {
		public ReceiveOnceBehaviour(Agent a, int millis, MessageTemplate mt) {
			super(a, millis, mt);
		}

		public void action() {
			super.action();
			getAgent().removeBehaviour(this);
		}
	}
}