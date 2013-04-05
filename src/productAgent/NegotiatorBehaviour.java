package productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SequentialBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
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

		// foreachProductionstep in object[]{
		AID aid = _productAgent.getAID();
		_productAgent.addBehaviour(new Conversation(aid, null));
		// }

	}

	private class Conversation extends SequentialBehaviour {
		private AID _aid;
		private ProductionStep _productionStep;

		public Conversation(AID aid, ProductionStep productionStep) {
			this._aid = aid;
			this._productionStep = productionStep;
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
						System.out.println("send message to: " + _aid);
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			});

			MessageTemplate template = MessageTemplate.and(
					MessageTemplate.MatchPerformative(ACLMessage.CONFIRM),
					MessageTemplate.MatchConversationId(ConversationId));
			addSubBehaviour(new receiveBehaviour(myAgent, 40000, template) {
				public void handle(ACLMessage msg) {
					if (msg == null)
						System.out.println("Timeout");
					else
						System.out.println("Received: " + msg);

				}
			});
		}

	}

	/*
	 * Starts the conversation with the given equiplet agent how many timeslots
	 * an step will take. Will initiate the converstion and then waits for the
	 * response.
	 */
	public int doConversationStepDuration(ProductionStep step, AID aid) {
		// Send the request with parameters
		// Check if it is a confirm.
		// if so, ask how long it will take
		// save yerr shit
		try {
			ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
			message.addReceiver(aid);
			message.setOntology("CanPerformStep");
			message.setContentObject(step);
			_productAgent.send(message);
			System.out.println("send message to: " + aid);
		} catch (Exception e) {
			e.printStackTrace();
		}

		// So much for the sending. Lets try to receive a response.

		ACLMessage receive = _productAgent.receive(MessageTemplate
				.MatchOntology("GetStepDuration"));
		if (receive != null && receive.getSender() == aid) {
			try {
				if (receive.getPerformative() == ACLMessage.CONFIRM) {
					return 1;
				}

			} catch (Exception e) {
				e.printStackTrace();
			}
		} else {
			block();
		}
		return -1;
	}

	// --- Methods to initialize ACLMessages -------------------

	private ACLMessage newMsg(int perf, String content, AID dest) {
		ACLMessage msg = newMsg(perf);
		if (dest != null)
			msg.addReceiver(dest);
		msg.setContent(content);
		return msg;
	}

	private ACLMessage newMsg(int perf) {
		ACLMessage msg = new ACLMessage(perf);
		msg.setConversationId(_productAgent.generateCID());
		return msg;
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