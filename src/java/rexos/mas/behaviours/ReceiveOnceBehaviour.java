/**
 * @file rexos/mas/behaviours/ReceiveOnceBehaviour.java
 * @brief Behaviour that receives a single message.
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
package rexos.mas.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

/**
 * <code>ReceiveOnceBehaviour</code> is are functionally equivalent to <code>ReceiveBehaviour</code> but terminate after the first message matching the template arrives or after the timeout.
 * If a message arrives it calls <code>handle()</code> method only for the first message in the queue matching the template.
 *
 * @author Peter Bonnema
 * 
 */
public abstract class ReceiveOnceBehaviour extends ReceiveBehaviour {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

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

	/**
	 * 
	 * @see rexos.mas.behaviours.ReceiveBehaviour#action()
	 *
	 */
	@Override
	public void action() {
		ACLMessage msg = getMessage();
		long wakeupTime = getWakeupTime();
		if(myAgent != null && (msg = myAgent.receive(getTemplate())) != null) {
			handle(msg);
			if(myAgent != null)
				myAgent.removeBehaviour(this);
		} else if(wakeupTime == 0) {
			block();
		} else {
			long dt = wakeupTime - System.currentTimeMillis();
			if (dt > 0)
				block(dt);
			else {
				handle((ACLMessage) null);
				if(myAgent != null)
					myAgent.removeBehaviour(this);
			}
		}
	}
}
