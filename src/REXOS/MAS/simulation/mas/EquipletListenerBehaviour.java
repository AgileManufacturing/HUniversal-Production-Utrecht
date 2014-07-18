package simulation.mas;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;

import simulation.util.Ontology;
import simulation.util.Pair;
import simulation.util.Parser;
import simulation.util.Position;
import simulation.util.ProductStep;
import simulation.util.Triple;
import simulation.util.Tuple;
import jade.core.AID;
import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;

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
		ACLMessage msg = equiplet.blockingReceive();
		if (msg != null) {
			System.out.printf("EA:%s received message [performative=%s, conversation=%s, content=%s, from=%s]\n", equiplet.getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent(), msg.getSender().getLocalName());

			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_ARRIVED)) {
					handleProductArrived(msg);
				}
				break;
			// inform of the simulation that something happened with the equiplet
			case ACLMessage.INFORM_REF:
				handleSimulationInform(msg.getContent(), msg.getSender());
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
			double window = question.second - time;

			List<Triple<Integer, Double, List<Pair<Double, Double>>>> answer = equiplet.canExecute(time, question.third);

			double load = equiplet.load(time, window);
			Position position = equiplet.getPosition();
			String content = Parser.parseCanExecuteAnswer(answer, load, position);

			// send can execute reply

			ACLMessage reply = message.createReply();
			reply.setContent(content);
			reply.setPerformative(ACLMessage.PROPOSE);
			equiplet.send(reply);

			System.out.printf("EA:%s sen reply %s\n", equiplet.getLocalName(), reply.getContent());

		} catch (JSONException e) {
			System.err.printf("EA:%s failed to parse can execute()\n", equiplet.getLocalName());
			System.err.printf("EA:%s %s", equiplet.getLocalName(), e.getMessage());
		}
	}

	private void handleScheduling(ACLMessage message) {
		try {
			// scheduling info = < Service, Criteria, time, deadline >
			Tuple<String, Map<String, Object>, Double, Double> data = Parser.parseSchedule(message.getContent());

			String service = data.first;
			Map<String, Object> criteria = data.second;
			double time = data.third;
			double deadline = data.fourth;

			boolean success = equiplet.schedule(message.getSender(), time, deadline, service, criteria);

			// send can execute reply
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			reply.setPerformative(success ? ACLMessage.CONFIRM : ACLMessage.DISCONFIRM);
			equiplet.send(reply);

			System.out.printf("EA:%s sen reply to %s  %s\n", equiplet.getLocalName(), reply.getAllReceiver().next(), reply.getContent());

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

	private void handleInformationRequest(String content, AID sender, String conversationId, String reply) {
		try {
			JSONObject json = new JSONObject();
			json.append("state", equiplet.getEquipletState());
			json.append("waiting", equiplet.getWaiting());
			json.append("scheduled", equiplet.getScheduled());
			json.append("executed", equiplet.getExecuted());
			json.append("executing", equiplet.getExecuting());

			// send information reply
			ACLMessage message = new ACLMessage(ACLMessage.INFORM);
			message.addReceiver(sender);
			message.setContent(json.toString());
			message.setConversationId(conversationId);
			message.setInReplyTo(reply);
			equiplet.send(message);
		} catch (JSONException e) {
			// TODO failed to construct reply
		}
	}

	@Deprecated
	private void handleSimulationInform(String content, AID sender) {
		try {
			JSONObject json = new JSONObject(content);

			if (json.has("notify") && json.has("time")) {
				double time = json.getDouble("time");

				String notify = json.getString("notify");
				if (notify.equals("finished")) {
					equiplet.notifyJobFinished(time);
				} else if (notify.equals("breakdown")) {
					equiplet.notifyBreakdown(time);
				} else if (notify.equals("repaired")) {
					equiplet.notifyRepaired(time);
				} else {
					// TODO not understood
				}
			} else {
				// TODO failed
			}
		} catch (JSONException e) {
			// TODO: handle exception
			// send failed inform request
		}
	}

}
