/**
 * @file EquipletAgent.java
 * @brief Dummy main agent for testing purposes. Starts up the different agents
 *        and sets the (for now) hardcoded information.
 * @date Created: 02-04-2013
 * 
 * @author Alexander Streng
 * 
 *         Copyright © 2013, HU University of Applied Sciences Utrecht. All
 *         rights reserved.
 * 
 *         Redistribution and use in source and binary forms, with or without
 *         modification, are permitted provided that the following conditions
 *         are met: - Redistributions of source code must retain the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer. - Redistributions in binary form must reproduce the above
 *         copyright notice, this list of conditions and the following
 *         disclaimer in the documentation and/or other materials provided with
 *         the distribution. - Neither the name of the HU University of Applied
 *         Sciences Utrecht nor the names of its contributors may be used to
 *         endorse or promote products derived from this software without
 *         specific prior written permission.
 * 
 *         THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *         "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *         LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *         A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *         UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *         INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *         (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *         SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *         HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *         STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *         IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *         POSSIBILITY OF SUCH DAMAGE.
 **/

package rexos.mas.testingAgents;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.WakerBehaviour;
import jade.lang.acl.ACLMessage;

import rexos.mas.data.ProductionStep;
import java.util.Random;

@SuppressWarnings("serial")
public class EquipletAgent extends Agent{
	private int _canPerformStepId;
	private ProductionStep _prodStep;

	public int getCanPerformStepId(){
		return _canPerformStepId;
	}

	public static boolean getRandomBoolean(){
		Random randomBool = new Random();
		return randomBool.nextBoolean();
	}

	public static int getRandomInt(int random){
		Random randomInt = new Random();
		return randomInt.nextInt(random);
	}

	@Override
	protected void setup(){
		try{
			Object[] args = getArguments();
			_canPerformStepId = (Integer) args[0];
			addBehaviour(new receiveMsgBehaviour());
		} catch(Exception e){
			System.out.println("EquipletAgent exited with: " + e);
			doDelete();
		}
	}

	// Behaviour for receiving messages and handling simultaniously(parallel)
	private class receiveMsgBehaviour extends CyclicBehaviour{
		ParallelBehaviour parBehav;

		private receiveMsgBehaviour(){
			parBehav = new ParallelBehaviour(ParallelBehaviour.WHEN_ALL);
		}

		@Override
		public void action(){
			ACLMessage msg = receive();
			if (msg != null){
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour(msg);
				parBehav.addSubBehaviour(behaviour);
			} else{
				block();
			}
			myAgent.addBehaviour(parBehav);
		}
	}

	private class WaitMsgBehaviour extends OneShotBehaviour{
		ACLMessage message;
		int delay;
		boolean debug = false;
		int randomSeed = 12000;
		ACLMessage msg;

		public WaitMsgBehaviour(ACLMessage msg){
			this.msg = msg;
		}

		@Override
		public void action(){
			try{
				String convid = msg.getConversationId();
				switch(msg.getOntology()){
				case "ScheduleStep":
					if (debug)
						System.out.println("EQ: PA -> "
								+ myAgent.getLocalName() + " Received query"
								+ " schedule");
					break;
				case "CanPerformStep":
					_prodStep = (ProductionStep) msg.getContentObject();
					if (debug)
						System.out.println("EQ: PA -> "
								+ myAgent.getLocalName() + " Received query"
								+ " if I can perform step: "
								+ _prodStep.getId());
					message = new ACLMessage(ACLMessage.DISCONFIRM);
					message.setOntology("CanPerformStep");
					message.addReceiver(msg.getSender());
					// Debugging. Set false to disconfirm the requested step
					if (getRandomBoolean()){
						message.setPerformative(ACLMessage.CONFIRM);
					}
					message.setConversationId(convid);
					delay = getRandomInt(randomSeed / 2);
					if (debug)
						System.out.println("EQ: " + myAgent.getLocalName()
								+ " will wait : " + delay / 2
								+ " ms before sending " + "CanPerformStep "
								+ _prodStep.getId());
					myAgent.addBehaviour(new WakerBehaviour(myAgent, delay){
						@Override
						public void handleElapsedTimeout(){
							if (true && delay > 10000)
								System.out.println("EQ: "
										+ myAgent.getLocalName()
										+ " will wait : " + delay / 2
										+ " ms before sending "
										+ "CanPerformStep " + _prodStep.getId());
							send(message);
						}
					});
					break;
				case "GetProductionDuration":
					_prodStep = (ProductionStep) msg.getContentObject();
					if (debug)
						System.out.println("EQ: PA -> "
								+ myAgent.getLocalName() + " Received query"
								+ " how long it would take to perform: "
								+ _prodStep.getId());
					message = new ACLMessage(ACLMessage.INFORM);
					message.setOntology("GetProductionDuration");
					long timeslots = getRandomInt(30);
					message.setContentObject(timeslots);
					message.addReceiver(msg.getSender());
					message.setConversationId(convid);
					message.setConversationId(convid);
					delay = getRandomInt(randomSeed);
					if (true && delay > 10000)
						System.out.println("EQ: " + myAgent.getLocalName()
								+ " waited : " + delay + " ms before sending "
								+ "GetProductionDuration " + _prodStep.getId());
					myAgent.addBehaviour(new WakerBehaviour(myAgent, delay){
						@Override
						public void handleElapsedTimeout(){
							if (debug)
								System.out.println("EQ: "
										+ myAgent.getLocalName() + " -> PA "
										+ " send msg GetProductionDuration ( "
										+ _prodStep.getId() + " ) after "
										+ delay + " ms.");
							send(message);
						}
					});
					break;
				}
			} catch(Exception e){
				System.out.println("Error at equipletagent : " + e);
			}
		}
	}
}