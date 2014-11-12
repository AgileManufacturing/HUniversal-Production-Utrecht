package MAS.simulation.mas.product;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import org.json.JSONException;

import MAS.simulation.util.Ontology;
import MAS.simulation.util.Pair;
import MAS.simulation.util.Parser;
import MAS.simulation.util.Tick;

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
		MessageTemplate template = MessageTemplate.or(MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_PROCESSING), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED)), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_DELAYED));
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
			Pair<Tick, Integer> information = Parser.parseProductProcessing(message.getContent());
			boolean confirmation = product.getCurrentStep().getIndex() == information.second;
			if (confirmation) {
				product.onProductProcessing(information.first);
			} else {
				System.err.printf("PA:%s received wrong product step index that is processing\n", myAgent.getLocalName());
				throw new RuntimeException("PA:" + myAgent.getLocalName() + " received wrong product step index " + information.second + " that is processing which should be: "
						+ product.getCurrentStep().getIndex() + "\n");
			}

			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.CONFIRM);
			reply.setContent(Parser.parseConfirmation(confirmation));
			myAgent.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse product processing information\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}

	private void handleProductStepFinished(ACLMessage message) {
		try {

			Pair<Tick, Integer> information = Parser.parseProductFinished(message.getContent());
			boolean confirmation = product.getCurrentStep().getIndex() == information.second;
			if (confirmation) {
				product.onProductStepFinished(information.first);
			} else {

				System.err.printf("PA:%s received wrong product step index that is processing\n", myAgent.getLocalName());
				throw new RuntimeException("PA:" + myAgent.getLocalName() + " received wrong product step index " + information.second + " that is processing which should be: "
						+ product.getCurrentStep().getIndex() + "\n");
			}

			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.CONFIRM);
			reply.setContent(Parser.parseConfirmation(confirmation));
			myAgent.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse product step finished information\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}

	private void handleProductStepDelayed(ACLMessage message) {
		try {
			Pair<Tick, Integer> information = Parser.parseProductDelayed(message.getContent());
			product.onProductDelayed(information.first);
			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.CONFIRM);
			reply.setContent(Parser.parseConfirmation(true));
			product.send(reply);
		} catch (JSONException e) {
			System.err.printf("PA:%s failed to parse step delayed\n", myAgent.getLocalName());
			System.err.printf("PA:%s %s", myAgent.getLocalName(), e.getMessage());
		}
	}
}
