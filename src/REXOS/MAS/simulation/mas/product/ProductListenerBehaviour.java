package simulation.mas.product;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.json.JSONException;

import simulation.util.Ontology;
import simulation.util.Parser;

public class ProductListenerBehaviour extends Behaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ProductAgent product;

	public ProductListenerBehaviour(ProductAgent product) {
		this.product = product;
	}

	@Override
	public void action() {
		MessageTemplate template = MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM), MessageTemplate.MatchPerformative(ACLMessage.CONFIRM)));
		ACLMessage msg = myAgent.blockingReceive(template);
		if (msg != null) {
			System.out.printf("PA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", myAgent.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());
			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_PROCESSING)) {
					handleProductStepProcessing(msg);
				} else if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_FINISHED)) {
					handleProductStepFinished(msg);
				} else if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_DELAYED)) {
					handleProductStepDelayed(msg);
				}
				break;
			default:
				break;
			}
		}
	}

	@Override
	public boolean done() {
		return false;
	}

	private void handleProductStepProcessing(ACLMessage message) {
		try {
			boolean confirmation = Parser.parseConfirmation(message.getContent());
			if (confirmation) {
				product.onProductProcessing();
			} else {
				System.err.printf("PA:%s failed to receive confirmation.\n", myAgent.getLocalName());
			}

			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(confirmation));
			myAgent.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse confirmation\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}

	private void handleProductStepFinished(ACLMessage message) {
		try {
			boolean confirmation = Parser.parseConfirmation(message.getContent());
			if (confirmation) {
				product.onProductStepFinished();
			} else {
				System.err.printf("PA:%s failed to receive confirmation.\n", myAgent.getLocalName());
			}

			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(confirmation));
			myAgent.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse confirmation\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}

	private void handleProductStepDelayed(ACLMessage message) {
		try {
			double start = Parser.parseProductDelayed(message.getContent());
			product.onProductDelayed(start);
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			product.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse step delayed///////////////////////////\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}
}
