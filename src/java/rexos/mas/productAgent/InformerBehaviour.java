/**
 * @file InformerBehaviour.java
 * @brief Behaviour in which the product agent communicates with the equiplet
 *        agents to check if they can perform steps with the desired parameters.
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

package rexos.mas.productAgent;

import rexos.libraries.log.Logger;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

/*
 * Version 1.1 Author: Alexander Streng Behaviour for negotiating with the
 * equipletagens to determine the viable equiplets for the scheduling algorithm.
 * Will first check whether the equiplet can perform the given step with the
 * given parameters. If the equipletagent returns with an confirm, the product
 * agent will ask for the duration of the operation ( in timeslots ).
 */
public class InformerBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;
	private ParallelBehaviour _par;
	private Product _product;
	private Production _production;
	private ProductionEquipletMapper _prodEQmap;
	private boolean _isDone;

	public InformerBehaviour(){
	}

	@SuppressWarnings("unused")
	@Override
	public void action(){
		_productAgent = (ProductAgent) myAgent;
		_product = this._productAgent.getProduct();
		_production = _product.getProduction();
		_prodEQmap = new ProductionEquipletMapper();
		_isDone = false;
		/*
		 * We want to have our conversations in parallel. We also only want to
		 * return when all child conversations are finished. So iterate through
		 * each step in our productionlist and create a conversation object. (
		 * as behaviour )
		 */
		SequentialBehaviour seq = new SequentialBehaviour();
		myAgent.addBehaviour(seq);
		_par = new ParallelBehaviour(ParallelBehaviour.WHEN_ALL);
		seq.addSubBehaviour(_par);
		for(ProductionStep stp : _production.getProductionSteps()){
			if (stp.getStatus() == StepStatusCode.EVALUATING){
				// adds the step to te new list (the one that will be returned
				// to the scheduler)
				_prodEQmap.addProductionStep(stp.getId());
				ProductionEquipletMapper s1 = _production
						.getProductionEquipletMapping();
				for(AID aid : _production.getProductionEquipletMapping()
						.getEquipletsForProductionStep(stp.getId()).keySet()){
					_par.addSubBehaviour(new Conversation(aid, stp, _prodEQmap));
				}
			}
		}
		// Lets set our production objects
		seq.addSubBehaviour(new OneShotBehaviour(){
			private static final long serialVersionUID = 1L;

			@Override
			public void action(){
				if (_par.done()){
					System.out.println("Done informing.");
					try{
						_production.setProductionEquipletMapping(_prodEQmap);
						_product.setProduction(_production);
						_productAgent.setProduct(_product);
						_isDone = true;
					} catch(Exception e){
						Logger.log(e);
					}
				} else{
					System.out.println("Not done informing.");
				}
			}
		});
	}

	public boolean isDone(){
		return this._isDone;
	}

	/*
	 * The conversation class holds the behaviour for a conversation with an
	 * equiplet agent. The sequentialbehaviour it extends consists of 4
	 * sequences. 1 - Inform if the equiplet can perform the step with the given
	 * parameters 2 - wait for an response. ( handles a 10 sec timeout ) 3 - if
	 * the response was a CONFIRM, ask about the duration. 4- waits for the
	 * response ( handles a 10 sec timeout ).
	 */
	private class Conversation extends SequentialBehaviour{
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
		private AID _aid;
		private ProductionStep _productionStep;
		private boolean debug = true;
		private ProductionEquipletMapper _prodEQmap;

		public Conversation(AID aid, ProductionStep productionStep,
				ProductionEquipletMapper pem){
			this._aid = aid;
			this._productionStep = productionStep;
			this._prodEQmap = pem;
		}

		/*
		 * (non-Javadoc)
		 * @see jade.core.behaviours.Behaviour#onStart() starts the conversation
		 */
		@Override
		public void onStart(){
			final String ConversationId = _productAgent.generateCID();
			final MessageTemplate msgtemplate = MessageTemplate
					.MatchConversationId(ConversationId);
			// 1 - Inform if the equiplet can perform the step with the given
			// parameters
			addSubBehaviour(new OneShotBehaviour(){
				/**
				 * 
				 */
				private static final long serialVersionUID = 1L;

				@Override
				public void action(){
					try{
						ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
						message.setConversationId(ConversationId);
						message.addReceiver(_aid);
						message.setOntology("CanPerformStep");
						message.setContentObject(_productionStep);
						_productAgent.send(message);
						if (debug){
							System.out.println("Querying: "
									+ _aid.getLocalName()
									+ " if he can perform step: "
									+ _productionStep.getId());
						}
					} catch(Exception e){
						Logger.log(e);
					}
				}
			});
			// 2 - wait for an response. ( handles a 10 sec timeout )
			addSubBehaviour(new ReceiveBehaviour(myAgent, 10000, msgtemplate){
				/**
				 * 
				 */
				private static final long serialVersionUID = 1L;

				@Override
				public void handle(ACLMessage msg){
					if (msg == null){
						if (debug){
							System.out.println("Productagent "
									+ myAgent.getLocalName()
									+ " TIMED OUT on waiting for "
									+ _aid.getLocalName() + " CanPerformStep: "
									+ _productionStep.getId());
						}
					} else{
						if (msg.getPerformative() == ACLMessage.CONFIRM){
							if (debug){
								System.out.println("Received CONFIRM from: "
										+ _aid.getLocalName()
										+ ". He can perform step: "
										+ _productionStep.getId());
							}
							// 3 - if the response was a CONFIRM, ask about the
							// duration.
							addSubBehaviour(new OneShotBehaviour(){
								/**
								 * 
								 */
								private static final long serialVersionUID = 1L;

								@Override
								public void action(){
									try{
										ACLMessage message = new ACLMessage(
												ACLMessage.REQUEST);
										message.setConversationId(ConversationId);
										message.addReceiver(_aid);
										message.setOntology("GetProductionDuration");
										message.setContentObject(_productionStep);
										_productAgent.send(message);
										if (debug){
											System.out
													.println("Querying: "
															+ _aid.getLocalName()
															+ " how long it would take to perform: "
															+ _productionStep
																	.getId());
										}
									} catch(Exception e){
										Logger.log(e);
									}
								}
							});
							// 4- waits for the response ( handles a 10 sec
							// timeout ).
							addSubBehaviour(new ReceiveBehaviour(myAgent,
									10000, msgtemplate){
								/**
										 * 
										 */
								private static final long serialVersionUID = 1L;

								@Override
								public void handle(
										ACLMessage msg){
									if (msg == null){
										if (debug){
											System.out
													.println("Productagent "
															+ myAgent
																	.getLocalName()
															+ " TIMED OUT on waiting for "
															+ _aid.getLocalName()
															+ " GetProductionDurationStep "
															+ _productionStep
																	.getId());
										}
									} else{
										try{
											if (msg.getPerformative() == ACLMessage.INFORM){
												long timeSlots = (Long) msg
														.getContentObject();
												if (debug){
													System.out
															.println("Received INFORM from: "
																	+ _aid.getLocalName()
																	+ ". It can perform step: "
																	+ _productionStep
																			.getId()
																	+ ". This step will take "
																	+ timeSlots
																	+ " timeslots.");
												}
												// Adds the equiplet to the
												// production step in
												// the mapper list.
												_prodEQmap
														.addEquipletToProductionStep(
																_productionStep
																		.getId(),
																_aid, timeSlots);
											}
										} catch(UnreadableException e){
											System.out
													.println("Error on receiving timeslots from: "
															+ _aid.getLocalName()
															+ " " + e);
										}
									}
								}
							});
						} else{
							if (debug){
								System.out.println("Received DISCONFIRM from: "
										+ _aid.getLocalName()
										+ ". It cant perform step: "
										+ _productionStep.getId());
							}
						}
					}
				}
			});
		}
	}
}
