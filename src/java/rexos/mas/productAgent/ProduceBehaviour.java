/**
 * @file ProduceBehaviour.java
 * @brief Behaviour in which the product agent remains during de execution of a
 *        step.
 * @date Created: 16-04-2013
 * 
 * @author Arno Derks
 * @author Theodoor de Graaff
 * @author Ricky van Rijn
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
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

package rexos.mas.productAgent;

import com.mongodb.BasicDBObject;

import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import rexos.libraries.log.Logger;
import rexos.mas.data.BehaviourStatus;
import rexos.mas.data.ProductStep;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

public class ProduceBehaviour extends Behaviour {
	private static final long serialVersionUID = 1L;
	private Production _production;

	private boolean _isDone = false;
	private boolean _isError = false;
	private boolean _isCompleted = false;

	private BehaviourCallback _bc;

	/**
	 * @param myAgent
	 */
	public ProduceBehaviour(Agent myAgent, BehaviourCallback bc) {
		super(myAgent);
		this._bc = bc;
	}

	@Override
	public void onStart() {
		try {
			_production = ((ProductAgent) myAgent).getProduct().getProduction();
			if (_production != null && _production.getProductionSteps() != null) {
				for (ProductionStep stp : _production.getProductionSteps()) {
					if (stp.getStatus() == StepStatusCode.PLANNED) {
						myAgent.addBehaviour(new ProducingReceiver(myAgent, -1,
								MessageTemplate.MatchAll(), stp, this));
					} else {
						this._isError = true;
					}
				}
			}
		} catch (Exception e) {
			Logger.log(e);
		}
	}

	@Override
	public void action() {
		try {
			if (this._isDone) {
				this._bc.handleCallback(BehaviourStatus.COMPLETED);
				this._isCompleted = true;
			} else if (this._isError) {
				this._bc.handleCallback(BehaviourStatus.ERROR);
				this._isCompleted = true;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	@Override
	public boolean done() {
		return this._isCompleted;
	}
	
	public void reportProductStatus(BehaviourStatus bs) {
		if(bs == BehaviourStatus.COMPLETED) {
			this._isDone = true;
		} else {
			this._isError = true;
		}
	}
}

class ProducingReceiver extends rexos.mas.behaviours.ReceiveBehaviour {
	ProductionStep ProductAgentstp;
	private ProduceBehaviour _pb;

	/**
	 * @param agnt
	 * @param millis
	 * @param msgtmplt
	 * @param stp
	 **/
	public ProducingReceiver(Agent agnt, int millis, MessageTemplate msgtmplt,
			ProductionStep stp, ProduceBehaviour pb) {
		super(agnt, millis, msgtmplt);
		this.ProductAgentstp = stp;
		this._pb = pb;
	}

	private static final long serialVersionUID = 1L;

	@Override
	public void handle(ACLMessage m) {
		try {
			if (m.getOntology() != null) {
				switch (m.getOntology()) {
				case "StartStepQuestion":
					/*
					 * Equiplet agent requests permission for executing product
					 * step. Product agent grants permission Currently I cannot
					 * think of any reason why it wouldn�t. But I�m sure
					 * there are reasons.
					 */
					ACLMessage reply = m.createReply();
					reply.setOntology("StartStep");
					myAgent.send(reply);
					break;
				case "StatusUpdate":
					ProductStep step = new ProductStep((BasicDBObject)m.getContentObject());
					ProductAgentstp.setStatus(step.getStatus());
					switch (step.getStatus()) {
					case IN_PROGRESS:
						// In progress
						break;
					case SUSPENDED_OR_WARNING:
						/*
						 * Equiplet agent informs the product agent that a
						 * problem was encountered, but that it�s working on a
						 * solution.
						 */
					case FAILED:
						/*
						 * Equiplet agent informs the product agent that the
						 * product step has been aborted or has failed,
						 * including a reason and source. Product agent
						 * reschedules or gives up entirely
						 */
						break;
					case DONE:
						/*
						 * Equiplet agent informs the product agent that the
						 * product step has been executed successfully.
						 */
						((ProductAgent) myAgent).getProduct()
								.addStatusDataToLog(m.getSender(),
										step.getStatusData());
						_pb.reportProductStatus(BehaviourStatus.COMPLETED);
						break;
					default:
						Logger.log(new UnsupportedOperationException(
								"No case for " + step.getStatus()));
						break;
					}
					break;
				case "EquipletAgentDied":
					/* EquipletAgent taken down */
					break;
				default:
					Logger.log(new UnsupportedOperationException("No case for "
							+ m.getOntology()));
					break;
				}
			} else {
				_pb.reportProductStatus(BehaviourStatus.ERROR);
			}
		} catch (Exception e) {
			Logger.log(e);
			_pb.reportProductStatus(BehaviourStatus.ERROR);
		}
	}
}
