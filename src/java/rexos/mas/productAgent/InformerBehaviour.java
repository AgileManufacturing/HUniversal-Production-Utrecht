/**
 * @file rexos/mas/productAgent/InformerBehaviour.java
 * @brief Behaviour in which the product agent communicates with the equiplet
 *        agents to check if they can perform steps with the desired parameters.
 * @date Created: 02-04-2013
 * 
 * @author Alexander Streng
 * @author Mike Schaap
 * 
 *         Copyright � 2013, HU University of Applied Sciences Utrecht. All
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

package rexos.mas.productAgent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;

import rexos.libraries.log.Logger;
import rexos.mas.data.BehaviourStatus;
import rexos.mas.data.LogLevel;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;
import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import jade.core.behaviours.ParallelBehaviour;



/*
 * Version 1.1 Author: Alexander Streng Behaviour for negotiating with the
 * equipletagens to determine the viable equiplets for the scheduling algorithm.
 * Will first check whether the equiplet can perform the given step with the
 * given parameters. If the equipletagent returns with an confirm, the product
 * agent will ask for the duration of the operation ( in timeslots ).
 */
public class InformerBehaviour extends Behaviour {

	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;
	private Product _product;
	private Production _production;
	private ProductionEquipletMapper _prodEQmap;

	private boolean _isDone = false;;
	private boolean _isError = false;
	private boolean _isCompleted = false;

	private BehaviourCallback _bc;

	private ParallelBehaviour _parBehaviour;

	private Queue<SubInformerBehaviour> _subInformerBehaviours;
	private int _subInformersCompleted = 0;
	private int _totalSubinformers = 0;

	// private HashMap<SubInformerBehaviour, Boolean> _subInformerBehaviours;

	private int _currentRunningSubInformerBehaviours = 0;
	private int MAX_RUNNING_SUB_BEHAVIOURS = 5;

	/**
	 * COnstructs the InformerBehavior
	 * @param myAgent
	 * @param bc
	 */
	public InformerBehaviour(Agent myAgent, BehaviourCallback bc) {
		super(myAgent);
		this._bc = bc;
		_subInformerBehaviours = new LinkedList<SubInformerBehaviour>();
	}

	/**
	 * Starts the behavior
	 */
	@Override
	public void onStart() {

		_productAgent = (ProductAgent) myAgent;
		_product = this._productAgent.getProduct();
		_production = _product.getProduction();
		_prodEQmap = new ProductionEquipletMapper();

		ArrayList<ProductionStep> productionSteps = _production
				.getProductionSteps();

		int productionStepCount = productionSteps.size();

		_parBehaviour = new ParallelBehaviour(productionStepCount);

		for (ProductionStep productionStep : productionSteps) {
			if (productionStep.getStatus() == StepStatusCode.EVALUATING || productionStep.getStatus() == StepStatusCode.RESCHEDULE) {
				_prodEQmap.addProductionStep(productionStep.getId());
				ProductionEquipletMapper pem = _production
						.getProductionEquipletMapping();
				if (pem != null) {
					// Process all the equiplets capable of executing the
					// desired productionStep
					HashMap<AID, Long> equipletList = pem
							.getEquipletsForProductionStep(productionStep
									.getId());
					if (equipletList != null && equipletList.size() > 0) {
						for (AID aid : equipletList.keySet()) {
							String convId = _productAgent.generateCID();
							productionStep.setConversationIdForEquiplet(aid,convId);
							// _parBehaviour.addSubBehaviour(new
							// Conversation(aid, productionStep, _prodEQmap));
							_subInformerBehaviours.add(new SubInformerBehaviour(myAgent,this, productionStep, aid));
							_totalSubinformers++;
						}
					} else {
						Logger.log(LogLevel.ERROR, "Can't find any equiplets that can execute this production step. Capability: "
								+ productionStep.getCapability());
					}
				} else {
					Logger.log(LogLevel.ERROR, "Can't find any equiplets that can execute this production step. Capability: "
							+ productionStep.getCapability());
				}
			} else {
				Logger.log(LogLevel.ERROR, "Can't process a productionStep which isn't in the evaluating state");
			}
		}

		myAgent.addBehaviour(_parBehaviour);
	}

	/**
	 * Constructs the actions performed by the informer behavior
	 */
	@Override
	public void action() {
		try {
			if (!_subInformerBehaviours.isEmpty()) 
			{
				if (_currentRunningSubInformerBehaviours < MAX_RUNNING_SUB_BEHAVIOURS) {
					SubInformerBehaviour sib = _subInformerBehaviours.poll();
					sib.restartTimer();
					_parBehaviour.addSubBehaviour(sib);
					_currentRunningSubInformerBehaviours++;
				}
			} 
			else 
			{
				if (_isDone) 
				{
					_production.setProductionEquipletMapping(_prodEQmap);
					_product.setProduction(_production);
					_productAgent.setProduct(_product);
					this._bc.handleCallback(BehaviourStatus.COMPLETED, null);
					_isCompleted = true;
				} 
				else if (_isError) 
				{
					this._bc.handleCallback(BehaviourStatus.ERROR, null);
					_isCompleted = true;
				}
			}
			//block();
		} catch (NullPointerException e) {
			Logger.log(LogLevel.ERROR, e);
		}
	}
	
	@Override 
	public void reset() {
		super.reset();
		_isDone = false;;
		_isError = false;
		_isCompleted = false;
		_subInformersCompleted = 0;
		_totalSubinformers = 0;
	}

	/**
	 * Returns true when the behavior is done
	 * @return
	 */
	@Override
	public boolean done() {
		return _isCompleted;
	}

	/**
	 * The subInformerbehavior let's the next subInformerbehavior know he has been completed
	 * @param bs
	 * @param subBehaviour
	 */
	public void callbackSubInformerBehaviour(BehaviourStatus bs,
			SubInformerBehaviour subBehaviour) 
	{
		if (bs == BehaviourStatus.COMPLETED) 
		{
			Logger.log(LogLevel.DEBUG, "Setting time slots for equiplet: " + subBehaviour.getTargetEquiplet() + " duration: " + subBehaviour.getTimeslotDuration());
			_prodEQmap.setTimeSlotsForEquiplet(subBehaviour.getProductionStepId(), subBehaviour.getTargetEquiplet(), subBehaviour.getTimeslotDuration());
		} 
		else 
		{
			Logger.log(LogLevel.ERROR, "callbackSubInformerBehaviour ended with error!");
		}
		
		_parBehaviour.removeSubBehaviour(subBehaviour);
		_currentRunningSubInformerBehaviours--;
		_subInformersCompleted++;
		
		if(_subInformersCompleted == _totalSubinformers) 
		{
			_isDone = true;
		}
	}
}