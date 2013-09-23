/**
 * @file rexos/mas/behaviours/ReceiveBehaviour.java
 * @brief Used for waiting for and responding to messages.
 * @date Created: 11 apr. 2013
 *
 * @author Peter Bonnema
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/
package agents.shared_behaviours;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * <code>ReceiveBehaviour</code> encapsulates the <code>receive()</code> method
 * and calls the <code>handle()</code> method for each message in the agents
 * message queue matching the specified <code>MessageTemplate</code> or after a
 * timeout. When no template is specified <code>handle()</code> is called for
 * all messages. When all messages have been processed or the timeout expires it
 * automatically continues to wait for new messages and the timer will restart.
 * Implement the <code>handle()</code> method to process the messages. To set
 * the timeout either supply a non-negative amount of milliseconds in the
 * constructor or call the <code>setTimeout()</code> method.
 * 
 * @author Peter
 * 
 */
public abstract class ReceiveBehaviour extends CyclicBehaviour {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var MessageTemplate template
	 * The template used to match messages.
	 */
	private MessageTemplate template;
	
	/**
	 * @var long timeout
	 * Timeout for receiving messages.
	 */
	private long timeout;
	
	/**
	 * @var long wakeupTime
	 * The time at which point the receive behaviour should wakeup to check for messages.
	 */
	private long wakeupTime;

	/**
	 * @var ACLMessage msg
	 * The last message that was received.
	 **/
	private ACLMessage msg;

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> without a
	 * <code>MessageTemplate</code> and no timeout (-1).
	 * 
	 * @param a
	 *            The agent this behaviour belongs to.
	 */
	public ReceiveBehaviour(Agent a) {
		this(a, -1, null);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> with the specified
	 * <code>MessageTemplate</code> and no timeout (-1).
	 * 
	 * @param a
	 *            The agent this behaviour belongs to.
	 * @param mt
	 *            The <code>MessageTemplate</code> to match the messages with.
	 */
	public ReceiveBehaviour(Agent a, MessageTemplate mt) {
		this(a, -1, mt);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> without a
	 * <code>MessageTemplate</code> and with the specified timeout.
	 * 
	 * @param a
	 *            The agent this behaviour belongs to.
	 * @param millis
	 *            The milliseconds for the timeout. millis < 0 means no timeout.
	 *            millis == 0 means that <code>handle()</code> will be called
	 *            immediately.
	 */
	public ReceiveBehaviour(Agent a, int millis) {
		this(a, millis, null);
	}

	/**
	 * Instantiates a new <code>ReceiveBehaviour</code> with the specified
	 * <code>MessageTemplate</code> and specified timeout.
	 * 
	 * @param a
	 *            The agent this behaviour belongs to.
	 * @param millis
	 *            The milliseconds for the timeout. millis < 0 means no timeout.
	 *            millis == 0 means that <code>handle()</code> will be called
	 *            immediately.
	 * @param mt
	 *            The <code>MessageTemplate</code> to match the messages with.
	 */
	public ReceiveBehaviour(Agent a, int millis, MessageTemplate mt) {
		super(a);
		timeout = millis;
		template = mt;
		restartTimer();
	}

	/**
	 * 
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action() {
		while (myAgent != null && (msg = myAgent.receive(template)) != null) {
			handle(msg);
			restartTimer();
		}
		if (wakeupTime == 0) {
			block();
		} else {
			long dt = wakeupTime - System.currentTimeMillis();
			if (dt > 0)
				block(dt);
			else {
				clearTimer();
				handle((ACLMessage) null);
			}
		}
	}

	/**
	 * Sets the timer to <code>millis</code> milliseconds in which a timeout
	 * should occur if no message matching the template were to arrive within
	 * that time then it restarts the timer.
	 * 
	 * @param millis
	 * 		The number of milliseconds in which a timeout should occur.
	 */
	public void setTimer(long millis) {
		timeout = millis;
		restartTimer();
	}

	/**
	 * Clears the timer.
	 */
	public void clearTimer() {
		timeout = -1;
		restartTimer();
	}

	/**
	 * Restarts the timer.
	 */
	public void restartTimer() {
		wakeupTime = (timeout < 0 ? 0 : System.currentTimeMillis() + timeout);
	}

	/**
	 * 
	 * @see jade.core.behaviours.SimpleBehaviour#reset()
	 */
	@Override
	public void reset() {
		super.reset();
		msg = null;
		timeout = -1;
		restartTimer();
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
	 * Gets the message template for this behaviour.
	 * 
	 * @return the template
	 */
	public MessageTemplate getTemplate() {
		return template;
	}

	/**
	 * Gets the timeout for this behaviour.
	 * 
	 * @return the timeout
	 */
	public long getTimeout() {
		return timeout;
	}

	/**
	 * Gets the wakeup time for this behaviour.
	 * 
	 * @return the wakeupTime
	 */
	public long getWakeupTime() {
		return wakeupTime;
	}

	/**
	 * This method is called when a message matching
	 * 
	 * @param message
	 *            the m
	 */
	public abstract void handle(ACLMessage message); /*
													 * can be redefined in
													 * sub_class
													 */
}