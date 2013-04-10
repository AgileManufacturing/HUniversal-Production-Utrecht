package behaviours;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * @author Peter
 * 
 * <code>ReceiveBehaviour</code>'s encapsulates the <code>receive()</code> method and calls the <code>handle()</code> method for each message in the agents message queue matching the specified <code>MessageTemplate</code> or after a timeout.
 * When no template
 * When all messages have been processed or the timeout expires it automatically continues to wait for new messages and the timeout will be reset. Extend this class and implement the <code>handle()</code> method to process the messages.
 * To set the timeout either supply an amount of milliseconds above 0 in the constructor call or call the <code>setTimeout()</code> method. Use the <code>clearTimeout()</code> to clear the timeout after it has been set.
 *
 */
@SuppressWarnings("serial")
abstract class ReceiveBehaviour extends CyclicBehaviour {

	private MessageTemplate template;
	private long timeout, wakeupTime;

	private ACLMessage msg;

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> without a <code>MessageTemplate</code> and no timeout.
	 *
	 * @param a The agent this behaviour belongs to.
	 */
	public ReceiveBehaviour(Agent a) {
		this(a, -1, null);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> with the specified <code>MessageTemplate</code> and no timeout.
	 *
	 * @param a The agent this behaviour belongs to.
	 * @param mt The <code>MessageTemplate</code> to match the messages with.
	 */
	public ReceiveBehaviour(Agent a, MessageTemplate mt) {
		this(a, -1, mt);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> without a <code>MessageTemplate</code> and with the specified timeout.
	 *
	 * @param a The agent this behaviour belongs to.
	 * @param millis The milliseconds for the timeout. millis < 0 means no timeout. millis == 0 means that <code>handle()</code> will be called immediately.
	 */
	public ReceiveBehaviour(Agent a, int millis) {
		this(a, millis, null);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> with the specified <code>MessageTemplate</code> and specified timeout.
	 *
	 * @param a The agent this behaviour belongs to.
	 * @param millis The milliseconds for the timeout. millis < 0 means no timeout. millis == 0 means that <code>handle()</code> will be called immediately.
	 * @param mt The <code>MessageTemplate</code> to match the messages with.
	 */
	public ReceiveBehaviour(Agent a, int millis, MessageTemplate mt) {
		super(a);
		timeout = millis;
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
			clearTimeout();
			handle((ACLMessage) null);
		}
	}

	/**
	 * Gets the current message.
	 *
	 * @return the message
	 */
	public ACLMessage getMessage() {
		return msg;
	}
	
	/**
	 * Sets the time in milliseconds in which a timeout should occur if no message matching the template were to arrive within that time.
	 * 
	 * @param millis
	 */
	public void setTimeout(long millis) {
		timeout = millis;
		onStart();
	}
	
	/**
	 * Resets the timeout. The <code>handle()</code> method will only be called upon arrival of messages matching the template.
	 */
	public void clearTimeout() {
		timeout = -1;
		onStart();
	}

	public void onStart() {
		wakeupTime = (timeout < 0 ? Long.MAX_VALUE : System.currentTimeMillis()
				+ timeout);
	}

	public void reset() {
		super.reset();
		msg = null;
		timeout = -1;
	}

	/**
	 * Reset the behaviour. Also resets the timeout and sets it to the specified value.
	 *
	 * @param timeout The timeout
	 */
	public void reset(int timeout) {
		reset();
		this.timeout = timeout;
	}

	/**
	 * This method is called when a message matching 
	 *
	 * @param m the m
	 */
	public abstract void handle(ACLMessage m); /* can be redefined in sub_class */
	
	/**
	 * @author Peter
	 * <code>ReceiveOnceBehaviour</code>'s are functionally equivalent to <code>ReceiveBehaviour</code> but terminate after the first message matching the template arrives or after the timeout.
	 * If a message arrives it still calls <code>handle()</code> for every matching message in the queue present at that time.
	 *
	 */
	public abstract class ReceiveOnceBehaviour extends ReceiveBehaviour {
		/**
		 * Instantiates a new <code>ReceiveOnceBehaviour</code> without a <code>MessageTemplate</code> and no timeout.
		 * 
		 * @param a
		 */
		public ReceiveOnceBehaviour(Agent a) {
			this(a, -1, null);
		}
		
		/**
		 * Instantiates a new <code>ReceiveOnceBehaviour</code> without a <code>MessageTemplate</code> with the specified timeout.
		 * 
		 * @param a
		 * @param millis
		 */
		public ReceiveOnceBehaviour(Agent a, int millis) {
			this(a, millis, null);
		}
		
		/**
		 * Instantiates a new <code>ReceiveOnceBehaviour</code> with the specified <code>MessageTemplate</code> and no timeout.
		 * 
		 * @param a
		 * @param mt
		 */
		public ReceiveOnceBehaviour(Agent a, MessageTemplate mt) {
			this(a, -1, mt);
		}
		
		/**
		 * Instantiates a new <code>ReceiveOnceBehaviour</code> with the specified <code>MessageTemplate</code> with the specified timeout.
		 * 
		 * @param a
		 * @param millis
		 * @param mt
		 */
		public ReceiveOnceBehaviour(Agent a, int millis, MessageTemplate mt) {
			super(a, millis, mt);
		}

		public void action() {
			super.action();
			getAgent().removeBehaviour(this);
		}
	}
}