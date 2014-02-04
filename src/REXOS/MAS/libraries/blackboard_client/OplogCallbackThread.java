/**
 * @file src/REXOS/MAS/libraries/blackboard_client/OplogCallbackThread.java
 * @brief Asynchronously handles callbacks.
 * @date Created: 28 jun. 2013
 *
 * @author Jan-Willem Willebrands
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
 **/
package libraries.blackboard_client;

import java.util.Stack;

import libraries.blackboard_client.data_classes.BlackboardSubscriber;
import libraries.blackboard_client.data_classes.OplogEntry;

/**
 * Simple data object used to store information about a callback.
 */
class Callback {
	/**
	 * @var BlackboardSubscriber subscriber
	 * The subscriber that should receive a callback.
	 */
	BlackboardSubscriber subscriber;
	
	/**
	 * @var OplogEntry entry
	 * The OplogEntry describing the event that triggered this callback.
	 */
	OplogEntry entry;
	
	/**
	 * Constructs a callback with the specified subscriber and entry.
	 * 
	 * @param subscriber The subscriber that should receive a callback.
	 * @param entry The OplogEntry describing the event that triggered this callback.
	 *
	 */
	public Callback(BlackboardSubscriber subscriber, OplogEntry entry) {
		this.subscriber = subscriber;
		this.entry = entry;
	}

	/**
	 * Returns the subscriber for this callback.
	 * @return The subscriber for this callback.
	 */
	public BlackboardSubscriber getSubscriber() {
		return subscriber;
	}

	/**
	 * Returns the entry for this callback.
	 * 
	 * @return The entry for this callback.
	 */
	public OplogEntry getEntry() {
		return entry;
	}
}

/**
 * Asynchronously handles callbacks.
 **/
public class OplogCallbackThread extends Thread {
	/**
	 * @var Stack<Callback> callbacks
	 * List of callbacks that need to be handled.
	 */
	private Stack<Callback> callbacks;
	
	/**
	 * @var boolean running
	 * Current running state of the callback thread.
	 */
	private boolean running = true;
	
	/**
	 * Constructs and starts a new thread.
	 */
	public OplogCallbackThread() {
		callbacks = new Stack<Callback>();
		start();
	}
	
	/**
	 * Adds a callback to the list and notifies the thread.
	 * 
	 * @param subscriber The subscriber that should receive a callback.
	 * @param entry The OplogEntry describing the event that triggered this callback.
	 *
	 */
	public synchronized void addCallback(BlackboardSubscriber subscriber, OplogEntry entry) {
		Callback callback = new Callback(subscriber, entry);
		callbacks.push(callback);
		notifyAll();
	}
	
	/**
	 * Sets the current running state to false and notifies the thread.
	 */
	public synchronized void shutdown() {
		running = false;
		notifyAll();
	}
	
	/**
	 * Waits for callbacks to be added and handles them.
	 **/
	@Override
	public synchronized void run() {
		while (running) {
			if (callbacks.empty()) {
				try {
					wait();
				} catch (InterruptedException ex) {}
			} else {
				while (!callbacks.empty()) {
					Callback callback = callbacks.pop();
					
					callback.getSubscriber().onMessage(callback.getEntry().getOperation(), callback.getEntry());
				}
			}
		}
	}
}
