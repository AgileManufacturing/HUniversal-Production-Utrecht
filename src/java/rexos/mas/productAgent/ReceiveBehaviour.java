/**
 * @file ReceiveBehaviour.java
 * @brief class based on the ReceiverBehaviour. instead of polling .done(), this
 *        class will return when either the timeout fires or a msg is received.
 * @date Created: 23-04-2013
 * 
 * @author Alexander Streng
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package java.rexos.mas.productAgent;

import jade.core.Agent;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

public class ReceiveBehaviour extends SimpleBehaviour{
	private static final long serialVersionUID = 1L;
	private MessageTemplate template;
	private long timeOut, wakeupTime;
	private boolean finished;
	private ACLMessage msg;

	public ACLMessage getMessage(){
		return msg;
	}

	public ReceiveBehaviour(Agent agnt, int millis, MessageTemplate msgtmplt){
		super(agnt);
		timeOut = millis;
		template = msgtmplt;
	}

	@Override
	public void onStart(){
		wakeupTime = (timeOut < 0 ? Long.MAX_VALUE : System.currentTimeMillis()
				+ timeOut);
	}

	@Override
	public boolean done(){
		return finished;
	}

	@Override
	public void action(){
		if (template == null)
			msg = myAgent.receive();
		else
			msg = myAgent.receive(template);
		if (msg != null){
			finished = true;
			handle(msg);
			return;
		}
		// wat is dt???
		long dt = wakeupTime - System.currentTimeMillis();
		if (dt > 0)
			block(dt);
		else{
			finished = true;
			handle(msg);
		}
	}

	/*
	 * This function will be called in the sub_class e.g. new
	 * ReceiveBehaviour(myAgent, 10000, template) { public void
	 * handle(ACLMessage msg) { if(msg == null) timeout expired handle msg
	 * stuff. } }
	 */
	public void handle(ACLMessage m){
	}

	@Override
	public void reset(){
		msg = null;
		finished = false;
		super.reset();
	}

	// wat is dt???
	public void reset(int dt){
		timeOut = dt;
		reset();
	}
}