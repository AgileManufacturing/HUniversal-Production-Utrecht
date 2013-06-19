/**
 * @file ProduceBehaviour.java
 * @brief Behaviour in which the product agent remains during de execution of a
 *        step.
 * @date Created: 16-04-2013
 * 
 * @author Arno Derks
 * @author Theodoor de Graaff
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

package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import rexos.libraries.log.Logger;
import rexos.mas.data.Product;
import rexos.mas.data.ProductStep;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

import java.util.HashMap;

public class ProduceBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	private Production _production;
	private ProductionEquipletMapper _prodEQMap;
	ProductionEquipletMapper s1;
	ACLMessage msg;

	@Override
	public void action(){
		try{
			_prodEQMap = new ProductionEquipletMapper();
			// retrieve the productstep
			for(ProductionStep stp : _production.getProductionSteps()){
				if (stp.getStatus() == StepStatusCode.PLANNED){
					// adds the step to te new list (the one that will be
					// returned to the scheduler)
					_prodEQMap.addProductionStep(stp.getId());
					s1 = _production.getProductionEquipletMapping();
					// retrieve the AID
					HashMap<AID, Long> equipletAndTimeslot = _production
							.getProductionEquipletMapping()
							.getEquipletsForProductionStep(stp.getId());
					// roep seq behav aan
					myAgent.addBehaviour(new newProducing(equipletAndTimeslot, stp));
				}
			}
		}
		catch(Exception e) {
			
		}
	}
}

class newProducing extends SequentialBehaviour{
	private static final long serialVersionUID = 1L;
	private HashMap<AID, Long> EqAndTs;
	private ProductionStep productionStep;

	public newProducing(HashMap<AID, Long> EqAndTs,
			ProductionStep productionStep){
		this.EqAndTs = EqAndTs;
		this.productionStep = productionStep;
	}

	@Override
	public void onStart(){
		addSubBehaviour(new OneShotBehaviour(){
			private static final long serialVersionUID = 1L;

			@Override
			public void action(){
				myAgent.addBehaviour(new receiveMsgBehaviour(EqAndTs,
						productionStep));
			}
		});
	}
}

class receiveMsgBehaviour extends CyclicBehaviour{
	private static final long serialVersionUID = 1L;
	HashMap<AID, Long> eqAndTs;
	private ProductionStep productionStep;

	receiveMsgBehaviour(HashMap<AID, Long> eqAndTs,
			ProductionStep productionStep){
		this.eqAndTs = eqAndTs;
		this.productionStep = productionStep;
	}

	@Override
	public void action(){
		ACLMessage msg = myAgent.receive();
		if (msg != null){
			myAgent.addBehaviour(new WaitMsgBehaviour(msg, eqAndTs,
					productionStep));
		} else{
			block();
		}
	}
}

class WaitMsgBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	ProductAgent _productAgent;
	ACLMessage msg;
	HashMap<AID, Long> eqAndTs;
	private ProductionStep productionStep;

	public WaitMsgBehaviour(ACLMessage msg, HashMap<AID, Long> eqAndTs,
			ProductionStep productionStep){
		this.msg = msg;
		this.eqAndTs = eqAndTs;
		this.productionStep = productionStep;
	}

	@Override
	public void action(){
		_productAgent = (ProductAgent) myAgent;
		SequentialBehaviour seq = new SequentialBehaviour();
		myAgent.addBehaviour(seq);
		msg = new ACLMessage(ACLMessage.INFORM);
		msg.setOntology("StartProduction");
		
		//AID CHECK
		if(eqAndTs.get(msg.getSender()) != productionStep.getId()){
			Logger.log(new UnsupportedOperationException("Equiplet sender not equal to associated equiplet in productionStep"));
		if (eqAndTs.get(msg.getSender()) != productionStep.getId()){
			Logger.log(new UnsupportedOperationException(
					"Equiplet sender not equal to associated equiplet in productionStep"));
		}
		//TODO add content to change step state to STATE_PRODUCING
		msg.addReceiver(msg.getSender());
		myAgent.send(msg);
	}
}
}

class producing extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	Product _product;
	ProductAgent _productAgent;
	ACLMessage msg;

	@Override
	public void action(){
		try{
			switch(msg.getOntology()){
			case "StartStepQuestion":
				/*
				 * Equiplet agent requests permission for executing product
				 * step. Product agent grants permission Currently I cannot
				 * think of any reason why it wouldn’t. But I’m sure there are
				 * reasons.
				 */
				msg.addReplyTo(msg.getSender());
				msg.setOntology("StartStep");
				myAgent.send(msg);
				break;
			case "Planned":
				// Planned?
				break;
			case "StatusUpdate":
				ProductStep ps = (ProductStep) msg.getContentObject();
				switch(msg.getContent()){
				case "INPROGRESS":
					// In progress
					break;
				case "SUSPENDED_OR_WARNING":
					/*
					 * Equiplet agent informs the product agent that a problem
					 * was encountered, but that it’s working on a solution.
					 */
				case "FAILED":
					/*
					 * Equiplet agent informs the product agent that the product
					 * step has been aborted or has failed, including a reason
					 * and source. Product agent reschedules or gives up
					 * entirely
					 */
					break;
				case "DONE":
					/*
					 * Equiplet agent informs the product agent that the product
					 * step has been executed successfully.
					 */
					_product.addStatusDataToLog(msg.getSender(), ps.getStatusData());
					break;
				default:
					Logger.log(new UnsupportedOperationException("No case for "
							+ msg.getContent()));
					break;
				}
				break;
			case "EquipletAgentDied":
				/* EquipletAgent taken down */
				break;
			default:
				Logger.log(new UnsupportedOperationException("No case for "
						+ msg.getOntology()));
				break;
			}
		} catch(Exception e){
			Logger.log(e);
		}
	}
}
