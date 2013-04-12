package productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.ParallelBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;

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

	@Override
	public void action() {
		_productAgent = (ProductAgent) myAgent;

		/*
		 * Testing. Hardcode list with eqa's this list will later be renderd
		 * from the planning behaviour.
		 */
		ProductionEquipletMapper pem = _productAgent.getProduct()
				.getProduction().getProductionEquipletMapping();

		for (ProductionStep stp : _productAgent.getProduct().getProduction()
				.getProductionSteps()) {

			pem.addProductionStep(stp.getId());

			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa1",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa2",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa3",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa4",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa5",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa6",
					AID.ISLOCALNAME));
		}

		/*
		 * We want to have our conversations in parallel. We also only want to
		 * return when all child conversations are finished. So iterate through
		 * each step in our productionlist and create a conversation object. (
		 * as behaviour )
		 */
		ParallelBehaviour par = new ParallelBehaviour(
				ParallelBehaviour.WHEN_ALL);

		for (ProductionStep stp : _productAgent.getProduct().getProduction()
				.getProductionSteps()) {
			for (AID aid : _productAgent.getProduct().getProduction()
					.getProductionEquipletMapping()
					.getEquipletsForProductionStep(stp.getId())) {
				par.addSubBehaviour(new Conversation(aid, stp));
			}
		}
		myAgent.addBehaviour(par);
	}

	public int onEnd() { 
		 	return 0; 
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
		private boolean debug = false;

		public Conversation(AID aid, ProductionStep productionStep) {
			this._aid = aid;
			this._productionStep = productionStep;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see jade.core.behaviours.Behaviour#onStart() starts the conversation
		 */
		public void onStart() {
			final String ConversationId = _productAgent.generateCID();
			final MessageTemplate template = MessageTemplate
					.MatchConversationId(ConversationId);

			// 1 - Inform if the equiplet can perform the step with the given
			// parameters
			addSubBehaviour(new OneShotBehaviour() {
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

			// 2 - wait for an response. ( handles a 10 sec timeout )
			addSubBehaviour(new ReceiveBehaviour(myAgent, 10000, template) {
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
								public void action() {
									try {
										ACLMessage message = new ACLMessage(
												ACLMessage.REQUEST);
										message.setConversationId(ConversationId);
										message.addReceiver(_aid);
										message.setOntology("GetProductionDuration");
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
												_productAgent
														.getProduct()
														.getProduction()
														.getProductionEquipletMapping()
														.addEquipletToProductionStep(
																_productionStep
																		.getId(),
																_aid);

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
