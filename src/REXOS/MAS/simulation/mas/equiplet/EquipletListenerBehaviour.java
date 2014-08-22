package simulation.mas.equiplet;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.List;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;

import simulation.mas.product.ProductStep;
import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.Triple;
import simulation.util.Tuple;

public class EquipletListenerBehaviour extends Behaviour {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private EquipletAgent equiplet;
	private boolean done;

	public EquipletListenerBehaviour(EquipletAgent equiplet) {
		this.equiplet = equiplet;
		this.done = false;
	}

	@Override
	public void action() {
		// Listen only possible incoming conversation ids, note that otherwise the simulation would jam as the listener receives messages that else where is waited upon
		// MessageTemplate template = MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM),
		// MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.CONFIRM), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED))));
		MessageTemplate template = MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED), MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_CAN_EXECUTE), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE)));
		ACLMessage msg = equiplet.blockingReceive(template);
		if (msg != null) {
			System.out.printf("EA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", equiplet.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());

			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_ARRIVED)) {
					handleProductArrived(msg);
				}
				break;
			// Request of other agent to get information to schedule a job
			// will send confirm or disconfirm message in return
			case ACLMessage.REQUEST:
				handleScheduling(msg);
				break;
			// query for information of the equiplet
			case ACLMessage.QUERY_REF:
				handleCanExecute(msg);
				break;
			case ACLMessage.QUERY_IF:
				handleInformationRequest(msg);
				break;
			default:
				break;
			}
		}
	}

	@Override
	public boolean done() {
		return done;
	}

	private void handleCanExecute(ACLMessage message) {
		try {
			// can the equiplet execute the Triple < from time, within deadline, the following product steps >
			Triple<Double, Double, List<ProductStep>> question = Parser.parseCanExecute(message.getContent());

			double time = question.first;
			double deadline = question.second;
			double window = deadline - time;

			List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer = equiplet.canExecute(time, deadline, question.third);

			double load = equiplet.load(time, window);
			Position position = equiplet.getPosition();
			String content = Parser.parseCanExecuteAnswer(answer, load, position);

			// send can execute reply

			ACLMessage reply = message.createReply();
			reply.setContent(content);
			reply.setPerformative(ACLMessage.PROPOSE);
			equiplet.send(reply);

			System.out.printf("EA:%s send reply to %s : %s\n", equiplet.getLocalName(), message.getSender().getLocalName(), reply.getContent());

		} catch (JSONException e) {
			System.err.printf("EA:%s failed to parse can execute()\n", equiplet.getLocalName());
			System.err.printf("EA:%s %s", equiplet.getLocalName(), e.getMessage());
		}
	}

	private void handleScheduling(ACLMessage message) {
		try {
			// scheduling info = List of product steps :: [< time, deadline, Service, Criteria >]
			List<Tuple<Double, Double, String, Map<String, Object>>> data = Parser.parseScheduleRequest(message.getContent());
			boolean success = equiplet.schedule(message.getSender(), data);

			// send can execute reply
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			reply.setPerformative(success ? ACLMessage.CONFIRM : ACLMessage.DISCONFIRM);
			equiplet.send(reply);

			System.out.printf("EA:%s send reply to %s  %s\n", equiplet.getLocalName(), message.getSender().getLocalName(), reply.getContent());
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to parse scheduling()\n", equiplet.getLocalName());
			System.err.printf("EA:%s %s", equiplet.getLocalName(), e.getMessage());
		}
	}

	private void handleProductArrived(ACLMessage message) {
		try {
			double time = Parser.parseProductArrived(message.getContent());
			equiplet.notifyProductArrived(message.getSender(), time);

			// send can reply
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			reply.setPerformative(ACLMessage.CONFIRM);
			equiplet.send(reply);
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to parse product arrived.\n", equiplet.getLocalName());
			System.err.printf("EA:%s %s", equiplet.getLocalName(), e.getMessage());
		}
	}

	private void handleInformationRequest(ACLMessage message) {
		try {
			JSONObject json = new JSONObject();
			json.append("state", equiplet.getEquipletState());
			json.append("waiting", equiplet.getWaiting());
			json.append("scheduled", equiplet.getScheduled());
			json.append("executed", equiplet.getExecuted());
			json.append("executing", equiplet.getExecuting());

			// send information reply
			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.INFORM);
			reply.setContent(json.toString());
			equiplet.send(message);
		} catch (JSONException e) {
			// TODO failed to construct reply
		}
	}
}
