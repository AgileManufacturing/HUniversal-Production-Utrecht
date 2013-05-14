/**
 * @file ProduceBehaviour.java
 * @brief Behaviour in which the product agent remains during de execution of a
 *        step.
 * @date Created: 16-04-2013
 * 
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

package productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import newDataClasses.LogMessage;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;
import newDataClasses.ProductionStepStatus;

public class ProduceBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	private Product _product;// TODO unused
	private Production _production;
	private ProductAgent _productAgent; // TODO unused
	private ProductionEquipletMapper _prodEQMap; // TODO unused
	private int currProdStep = 1;
	ProductionEquipletMapper s1;
	ACLMessage msg;

	@SuppressWarnings("unused")
	private class receiveMsgBehaviour extends CyclicBehaviour{
		private static final long serialVersionUID = 1L;

		private receiveMsgBehaviour(){
		}

		@Override
		public void action(){
			@SuppressWarnings("hiding")
			ACLMessage msg = myAgent.receive();
			if (msg != null){
				WaitMsgBehaviour behaviour = new WaitMsgBehaviour(msg);
			} else{
				block();
			}
		}
	}

	private class WaitMsgBehaviour extends OneShotBehaviour{
		private static final long serialVersionUID = 1L;
		ACLMessage msg;

		public WaitMsgBehaviour(ACLMessage msg){
			this.msg = msg;
		}

		@Override
		public void action(){
			_prodEQMap = new ProductionEquipletMapper();
			for(ProductionStep stp : _production.getProductionSteps()){
				if (stp.getStatus() == ProductionStepStatus.STATE_TODO){
					// adds the step to te new list (the one that will be
					// returned
					// to the scheduler)
					_prodEQMap.addProductionStep(stp.getId());
					s1 = _production.getProductionEquipletMapping();
					// TODO implementeren versie Ricky
					for(AID aid : _production.getProductionEquipletMapping()
							.getEquipletsForProductionStep(stp.getId())
							.keySet()){
						//
					}
				}
			}
			HashMap<Integer, HashMap<AID, Long>> bla = s1.getHashMap();
			try{
				switch(msg.getOntology()){
				// The productionstep has been initiated.
				case "productionStepStarted":
					// TODO key = msg.getContent().parse
					int key = 0; // temp
					if (key != currProdStep){
						// TODO error
					}
					if (!bla.get(key).containsKey(msg.getSender())){
						// TODO error
					}
					{
						ArrayList<ProductionStep> ProductionStepArrayList = ((ProductAgent) myAgent)
								.getProduct().getProduction()
								.getProductionSteps();
						for(ProductionStep stp : ProductionStepArrayList){
							if (key == stp.getId()){
								canProductionStepStart(stp);
							}
						}
					}
					break;
				// The productionstep has completed.
				case "productionStepFinished":
					// TODO key = msg.getContent().parse
					int keyfinish = 0; // temp
					if (keyfinish != currProdStep){
						// TODO error
					}
					if (!bla.get(keyfinish).containsKey(msg.getSender())){
						// TODO error
					}
					{
						ArrayList<ProductionStep> ProductionStepArrayList = ((ProductAgent) myAgent)
								.getProduct().getProduction()
								.getProductionSteps();
						for(ProductionStep stp : ProductionStepArrayList){
							if (keyfinish == stp.getId()){
								productionStepEnded(stp, true, null);
								// productionStepEnded(stp, msg.getContent,
								// msg.getContent);
							}
						}
					}
					currProdStep++;
					break;
				// For some reason production can't be started thus it has to be
				// rescheduled.
				case "notStarted":
					_productAgent.reschedule();
					break;
				default:
					break;
				}
			} catch(Exception e){
				System.out.println("" + e);
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action(){
		_productAgent = (ProductAgent) myAgent;
		SequentialBehaviour seq = new SequentialBehaviour();
		myAgent.addBehaviour(seq);
		@SuppressWarnings("hiding")
		ACLMessage msg = new ACLMessage(ACLMessage.INFORM);
		msg.setOntology("StartProduction");
		msg.addReceiver(null); // add the equiplet AID
		myAgent.send(msg);
	}

	static void canProductionStepStart(ProductionStep step){
		step.setStatus(ProductionStepStatus.STATE_PRODUCING);
	}

	void productionStepEnded(ProductionStep step, boolean succes,
			List<LogMessage> log){
		_product.addLogMsg(log);
		if (succes){
			step.setStatus(ProductionStepStatus.STATE_DONE);
		} else{
			step.setStatus(ProductionStepStatus.STATE_FAILED);
		}
	}
}
