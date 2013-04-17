package behaviours;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * <code>ReceiveBehaviour</code> encapsulates the <code>receive()</code> method and calls the <code>handle()</code> method for each message in the agents message queue matching the specified <code>MessageTemplate</code> or after a timeout.
 * When no template is specified all messages are removed from the queue and <code>handle()</code> is called for each of them.
 * When all messages have been processed or the timeout expires it automatically continues to wait for new messages and the timeout will restart. Extend this class and implement the <code>handle()</code> method to process the messages.
 * To set the timeout either supply a non-negative amount of milliseconds in the constructor or call the <code>setTimeout()</code> method. Use the <code>clearTimeout()</code> to clear the timeout.
 * 
 * @author Peter
 *
 */
public abstract class ReceiveBehaviour extends CyclicBehaviour {
	private static final long serialVersionUID = 1L;
	
	protected MessageTemplate template;
	protected long timeout, wakeupTime;

	protected ACLMessage msg;

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
		restartTimer();
	}

	public void action() {
		while(myAgent != null && (msg = myAgent.receive(template)) != null) {
			handle(msg);
			restartTimer();
		}
		if(wakeupTime == 0) {
			block();
		} else {
			long dt = wakeupTime - System.currentTimeMillis();
			if (dt > 0)
				block(dt);
			else {
				clearTimeout();
				handle((ACLMessage) null);
			}
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
		restartTimer();
	}
	
	/**
	 * Resets the timeout. The <code>handle()</code> method will only be called upon arrival of messages matching the template.
	 */
	public void clearTimeout() {
		timeout = -1;
		restartTimer();
	}
	
	/**
	 * Restarts the timer with the previously specified amount of milliseconds.
	 */
	public void restartTimer() {
		wakeupTime = (timeout < 0 ? 0 : System.currentTimeMillis()
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
	 * @param message the m
	 */
	public abstract void handle(ACLMessage message); /* can be redefined in sub_class */
}