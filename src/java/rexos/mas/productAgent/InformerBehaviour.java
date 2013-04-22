package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.ProductionStepStatus;

/*
 * Version 1.1
 * 
 * Author: Alexander Streng
 * 
 * Behaviour for negotiating with the equipletagens to
 * determine the viable equiplets for the scheduling algorithm.
 * Will first check whether the equiplet can perform the given step with the given parameters.
 * If the equipletagent returns with an confirm, the product agent will ask for the duration of
 * the operation ( in timeslots ).
 * 
 */
@SuppressWarnings("serial")
public class InformerBehaviour extends OneShotBehaviour {

	private ProductAgent _productAgent;
	private ParallelBehaviour _par;
	private Product _product;
	private Production _production;
	private ProductionEquipletMapper _pem;
	private boolean _isDone;

	public InformerBehaviour() {
	}

	@Override
	public void action() {
		_productAgent = (ProductAgent) myAgent;
		_product = this._productAgent.getProduct();
		_production = _product.getProduction();
		_pem = new ProductionEquipletMapper();
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

		for (ProductionStep stp : _product.getProduction().getProductionSteps()) {
			if (stp.getStatus() == ProductionStepStatus.STATE_TODO) {
				
				//adds the step to te new list (the one that will be returned to the scheduler)
				_pem.addProductionStep(stp.getId());

				for (AID aid : _productAgent.getProduct().getProduction()
						.getProductionEquipletMapping()
						.getEquipletsForProductionStep(stp.getId()).keySet()) {
					_par.addSubBehaviour(new Conversation(aid, stp, _pem));
				}
			}
		}

		// Checking if timeout expired?
		seq.addSubBehaviour(new OneShotBehaviour() {
			@Override
			public void action() {
				// TODO Auto-generated method stub
				if (_par.done()) {
					System.out.println("Done informing.");
					try {
						_production.setProductionEquipletMapping(_pem);
						_product.setProduction(_production);
						_productAgent.setProduct(_product);
						_isDone = true;
					} catch (Exception e) {
						e.printStackTrace();
					}
				} else {
					System.out.println("Not done informing.");
				}
			}
		});
	}

	public boolean isDone() {
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
	private class Conversation extends SequentialBehaviour {
		private AID _aid;
		private ProductionStep _productionStep;
		private boolean debug = true;
		private ProductionEquipletMapper _pem;

		public Conversation(AID aid, ProductionStep productionStep,
				ProductionEquipletMapper pem) {
			this._aid = aid;
			this._productionStep = productionStep;
			this._pem = pem;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see jade.core.behaviours.Behaviour#onStart() starts the conversation
		 */
		@Override
		public void onStart() {
			final String ConversationId = _productAgent.generateCID();
			final MessageTemplate template = MessageTemplate
					.MatchConversationId(ConversationId);

			// 1 - Inform if the equiplet can perform the step with the given
			// parameters
			addSubBehaviour(new OneShotBehaviour() {
				@Override
				public void action() {
					try {
						ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
						message.setConversationId(ConversationId);
						message.addReceiver(_aid);
						message.setOntology("CanPerformStep");
						message.setContentObject(_productionStep);
						_productAgent.send(message);
						if (debug) {
							System.out.println("Querying: "
									+ _aid.getLocalName()
									+ " if he can perform step: "
									+ _productionStep.getId());
						}
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			});

			// 2 - wait for an response. ( handles a 20 sec timeout )
			addSubBehaviour(new ReceiveBehaviour(myAgent, 20000, template) {
				@Override
				public void handle(ACLMessage msg) {
					if (msg == null) {

						if (debug) {
							System.out.println("Productagent "
									+ myAgent.getLocalName()
									+ " TIMED OUT on waiting for "
									+ _aid.getLocalName() + " CanPerformStep: "
									+ _productionStep.getId());
						}

					} else {

						if (msg.getPerformative() == ACLMessage.CONFIRM) {
							if (debug) {
								System.out.println("Received CONFIRM from: "
										+ _aid.getLocalName()
										+ ". He can perform step: "
										+ _productionStep.getId());
							}

							// 3 - if the response was a CONFIRM, ask about the
							// duration.
							addSubBehaviour(new OneShotBehaviour() {
								@Override
								public void action() {
									try {
										ACLMessage message = new ACLMessage(
												ACLMessage.REQUEST);
										message.setConversationId(ConversationId);
										message.addReceiver(_aid);
										message.setOntology("GetProductStepDuration");
										message.setContentObject(_productionStep);
										_productAgent.send(message);
										if (debug) {
											System.out
													.println("Querying: "
															+ _aid.getLocalName()
															+ " how long it would take to perform: "
															+ _productionStep
																	.getId());
										}
									} catch (Exception e) {
										e.printStackTrace();
									}
								}
							});

							// 4- waits for the response ( handles a 10 sec
							// timeout ).
							addSubBehaviour(new ReceiveBehaviour(myAgent,
									10000, template) {
								@Override
								public void handle(ACLMessage msg) {
									if (msg == null) {
										if (debug) {
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
									} else {
										try {
											if (msg.getPerformative() == ACLMessage.INFORM) {
												long timeSlots = (Long) msg
														.getContentObject();
												if (debug) {
													System.out
															.println("Received INFORM from: "
																	+ _aid.getLocalName()
																	+ ". He can perform step: "
																	+ _productionStep
																			.getId()
																	+ ". This step will take "
																	+ timeSlots
																	+ " timeslots.");
												}

												// Adds the equiplet to the
												// production step in
												// the mapper list.
												_pem.addEquipletToProductionStep(
														_productionStep.getId(),
														_aid, timeSlots);
											}
										} catch (UnreadableException e) {
											System.out
													.println("Error on receiving timeslots from: "
															+ _aid.getLocalName()
															+ " " + e);
										}
									}
								}
							});

						} else {
							if (debug) {
								System.out.println("Received DISCONFIRM from: "
										+ _aid.getLocalName()
										+ ". He cant perform step: "
										+ _productionStep.getId());
							}
						}
					}
				}
			});
		}
	}

}
