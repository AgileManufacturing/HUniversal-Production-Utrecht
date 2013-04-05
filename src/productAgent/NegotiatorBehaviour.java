package productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;

@SuppressWarnings("serial")
public class NegotiatorBehaviour extends CyclicBehaviour {

	private ProductAgent _productAgent;

	public NegotiatorBehaviour(ProductAgent pa) {
		this._productAgent = pa;
	}

	@Override
	public void action() {

		int timeSlots = -1;

		/*
		 * Testing. Hardcode list with eqa's
		 */
		ProductionEquipletMapper pem = new ProductionEquipletMapper();
		for (ProductionStep stp : _productAgent.getProduct().getProduction()
				.getProductionSteps()) {
			pem.addProductionStep(stp.getId());
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa1",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa2",
					AID.ISLOCALNAME));
			pem.addEquipletToProductionStep(stp.getId(), new AID("eqa3",
					AID.ISLOCALNAME));
		}
		// foreachProductionstep in object[]{
		for (ProductionStep stp : _productAgent.getProduct().getProduction()
				.getProductionSteps()) {
			for (AID aid : pem.getEquipletsForProductionStep(stp.getId())) {
				_productAgent.addBehaviour(new Conversation(aid, stp));
			}
		}
		// }
	}

	private class Conversation extends SequentialBehaviour {
		private AID _aid;
		private ProductionStep _productionStep;
		private boolean canPerformStep;

		public Conversation(AID aid, ProductionStep productionStep) {
			this._aid = aid;
			this._productionStep = productionStep;
			canPerformStep = false;
		}

		public void onStart() {
			final String ConversationId = _productAgent.generateCID();

			addSubBehaviour(new OneShotBehaviour() {
				public void action() {
					try {
						ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
						message.setConversationId(ConversationId);
						message.addReceiver(_aid);
						message.setOntology("CanPerformStep");
						message.setContentObject(_productionStep);
						_productAgent.send(message);

						System.out.println("Querying: " + _aid
								+ " if he can perform step: "
								+ _productionStep.getId());
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			});

			MessageTemplate template = MessageTemplate.and(
					MessageTemplate.MatchPerformative(ACLMessage.CONFIRM),
					MessageTemplate.MatchConversationId(ConversationId));

			addSubBehaviour(new receiveBehaviour(myAgent, 10000, template) {
				public void handle(ACLMessage msg) {
					if (msg == null)
						System.out.println("Productagent "
								+ myAgent.getLocalName()
								+ " timed out on waiting for CanPerformStep");
					// Dont add. Skip next step.
					else {
						// Add next behaviour
						canPerformStep = true;
						System.out.println("Received CONFIRM from: " + _aid
								+ ". He can perform step: "
								+ _productionStep.getId());
					}
				}
			});
			if (canPerformStep) {
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
							System.out.println("Querying: " + _aid
									+ " how long it would take to perform: "
									+ _productionStep.getId());
						} catch (Exception e) {
							e.printStackTrace();
						}
					}
				});

				template = MessageTemplate.and(
						MessageTemplate.MatchPerformative(ACLMessage.INFORM),
						MessageTemplate.MatchConversationId(ConversationId));

				addSubBehaviour(new receiveBehaviour(myAgent, 10000, template) {
					public void handle(ACLMessage msg) {
						if (msg == null)
							System.out
									.println("Productagent "
											+ myAgent.getLocalName()
											+ " timed out on waiting for GetProductionDuration");
						else {
							try {
								Long timeSlots = (Long) msg.getContentObject();
								System.out.println("Received INFORM from: "
										+ _aid + ". He can perform step: "
										+ _productionStep.getId()
										+ ". This step will take " + timeSlots
										+ " timeslots.");
							} catch (UnreadableException e) {
								System.out
										.println("Error on receiving timeslots from: "
												+ _aid + " " + e);
							}
						}
					}
				});
			}
		}

	}
}

@SuppressWarnings("serial")
class receiveBehaviour extends SimpleBehaviour {

	private MessageTemplate template;
	private long timeOut, wakeupTime;
	private boolean finished;

	private ACLMessage msg;

	public ACLMessage getMessage() {
		return msg;
	}

	public receiveBehaviour(Agent a, int millis, MessageTemplate mt) {
		super(a);
		timeOut = millis;
		template = mt;
	}

	public void onStart() {
		wakeupTime = (timeOut < 0 ? Long.MAX_VALUE : System.currentTimeMillis()
				+ timeOut);
	}

	public boolean done() {
		return finished;
	}

	public void action() {
		if (template == null)
			msg = myAgent.receive();
		else
			msg = myAgent.receive(template);

		if (msg != null) {
			finished = true;
			handle(msg);
			return;
		}
		long dt = wakeupTime - System.currentTimeMillis();
		if (dt > 0)
			block(dt);
		else {
			finished = true;
			handle(msg);
		}
	}

	public void handle(ACLMessage m) { /* can be redefined in sub_class */
	}

	public void reset() {
		msg = null;
		finished = false;
		super.reset();
	}

	public void reset(int dt) {
		timeOut = dt;
		reset();
	}

}