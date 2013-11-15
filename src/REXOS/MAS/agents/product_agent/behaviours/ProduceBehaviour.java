/**
 * @file rexos/mas/productAgent/ProduceBehaviour.java
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

package agents.product_agent.behaviours;

import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.HashMap;

import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.BehaviourStatus;
import agents.data_classes.ProductStep;
import agents.data_classes.Production;
import agents.data_classes.ProductionStep;
import agents.product_agent.BehaviourCallback;
import agents.product_agent.ProductAgent;

import com.mongodb.BasicDBObject;

public class ProduceBehaviour extends agents.shared_behaviours.ReceiveBehaviour {
	private static final long serialVersionUID = 1L;
	private Production _production;

	private BehaviourCallback _bc;

	private int _productionStepsCompleted = 0;
	private int _productionStepsCount = 0;
	private boolean _stopProduceBehaviour = false;
	private int _prodCounters = 0;
	private ProductAgent _productAgent;

	private HashMap<String, ProductionStep> _conversationIdToProductionStep;

	/**
	 * Constructs the produce behavior
	 * @param myAgent
	 */
	public ProduceBehaviour(Agent myAgent, BehaviourCallback bc) {
		this(myAgent, bc, MessageTemplate.or(MessageTemplate
				.MatchOntology("StatusUpdate"), MessageTemplate.or(
				MessageTemplate.MatchOntology("StartStepQuestion"),
				MessageTemplate.MatchOntology("EquipletAgentDied"))));

	}

	/**
	 * Constructs the produce behavior
	 * @param myAgent
	 * @param bc
	 * @param template
	 */
	public ProduceBehaviour(Agent myAgent, BehaviourCallback bc,
			MessageTemplate template) {
		super(myAgent, -1, template);
		this._bc = bc;
		_productAgent = (ProductAgent)myAgent;
	}

	/**
	 * Starts the produce algorithm
	 */
	@Override
	public void onStart() {
		
		_production = ((ProductAgent) myAgent).getProduct().getProduction();
		_conversationIdToProductionStep = _production
				.createConversationIdToProductionStepMapping();
		_productionStepsCount = _production.getProductionCount();
		
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.behaviours.ReceiveBehaviour#handle(jade.lang.acl.ACLMessage)
	 */
	/**
	 * handles the messages the behavior receives
	 * @param m
	 */
	@Override
	public void handle(ACLMessage m) {
		try {
			String conversationId = m.getConversationId();
			ProductionStep prodStep = _conversationIdToProductionStep
					.get(conversationId);
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
					ProductStep step = new ProductStep(
							(BasicDBObject) m.getContentObject());
					//prodStep.setStatus(step.getStatus());
					switch (step.getStatus()) {
					case WAITING:
						// Waiting
						break;
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
						_bc.handleCallback(BehaviourStatus.ERROR, null);
						break;
					case DONE:
						/*
						 * Equiplet agent informs the product agent that the
						 * product step has been executed successfully.
						 */
						((ProductAgent) myAgent).getProduct()
								.addStatusDataToLog(m.getSender(),
										step.getStatusData());
						Logger.log(LogLevel.DEBUG, "Completed productionStep :"+_prodCounters++);
						_productionStepsCompleted++;
						break;
					default:
						Logger.log(LogLevel.ERROR, "", new UnsupportedOperationException(
								"No case for " + step.getStatus()));
						break;
					}
					break;
				case "EquipletAgentDied":
					/* EquipletAgent taken down */
					break;
				default:
					Logger.log(LogLevel.ERROR, "", new UnsupportedOperationException("No case for "
							+ m.getOntology()));
					break;
				}
			} else {
				Logger.log(LogLevel.DEBUG, "No ontology set!");
			}
		} catch (Exception e) {
			Logger.log(LogLevel.ERROR, "", e);
			//_bc.handleCallback(BehaviourStatus.ERROR, null);
			//_stopProduceBehaviour = true;
		}
		if(_productionStepsCompleted == _productionStepsCount) {
			_bc.handleCallback(BehaviourStatus.COMPLETED, null);
			_stopProduceBehaviour = true;
		}
		if(_stopProduceBehaviour) {
			myAgent.removeBehaviour(this);
		}
	}
}
