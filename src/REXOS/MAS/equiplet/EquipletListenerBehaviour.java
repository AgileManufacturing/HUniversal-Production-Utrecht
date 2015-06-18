package MAS.equiplet;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.List;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.util.Ontology;
import MAS.util.Pair;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Triple;
import MAS.util.Tuple;

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
		// Listen only possible incoming conversation ids, note that otherwise the simulation would jam as the listener
		// receives messages that else where is waited upon
		// MessageTemplate template =
		// MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM),
		// MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.CONFIRM),
		// MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED))));
		MessageTemplate template = MessageTemplate.or(
				MessageTemplate.or(
						MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED), MessageTemplate.or(
								MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_RELEASE), MessageTemplate.or(
										MessageTemplate.MatchConversationId(Ontology.CONVERSATION_CAN_EXECUTE), MessageTemplate.or(
												MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE), MessageTemplate.or(
														MessageTemplate.MatchConversationId(Ontology.CONVERSATION_GET_DATA), MessageTemplate.or(
														MessageTemplate.MatchConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND),
														MessageTemplate.MatchConversationId(Ontology.CONVERSATION_LISTENER_COMMAND))))))),
				MessageTemplate.MatchConversationId(Ontology.CONVERSATION_INFORMATION_REQUEST)
		);
		ACLMessage msg = equiplet.blockingReceive(template);
		if (msg != null) {
			System.out.printf("EA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s]\n", equiplet.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent());

			switch (msg.getPerformative()) {
			case ACLMessage.INFORM:
				if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_ARRIVED)) {
					handleProductArrived(msg);
				} else if (msg.getConversationId().equals(Ontology.CONVERSATION_PRODUCT_RELEASE)) {
					handleProductRelease(msg);
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
				if(msg.getConversationId().equals(Ontology.CONVERSATION_GET_DATA)){
					//Handle get data requests
					equiplet.passEquipletGetRequest(msg);
				}else{
					//Handle information (no idea what conversation id is used here, it didn't have one to begin with..)
					handleInformationRequest(msg);
				}
				break;
			case ACLMessage.PROPOSE:
				if(msg.getConversationId().equals(Ontology.CONVERSATION_EQUIPLET_COMMAND)){
					//Handle equiplet command functie
					equiplet.passEquipletCommand(msg);
				}else if(msg.getConversationId().equals(Ontology.CONVERSATION_LISTENER_COMMAND)){
					//handle listener command functie
					equiplet.passOnChangedCommand(msg);
				}
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
			Triple<Tick, Tick, List<Triple<Integer, String, JSONObject>>> question = Parser.parseCanExecute(message.getContent());

			Tick time = question.first;
			Tick deadline = question.second;
			Tick window = deadline.minus(time);

			List<Triple<Integer, Tick, List<Pair<Tick, Tick>>>> answer = equiplet.canExecute(time, deadline, question.third);

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
			List<Tuple<Integer, Pair<Tick, Tick>, String, JSONObject>> data = Parser.parseScheduleRequest(message.getContent());
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

	/**
	 * handle the information from a product agent that he is arrived by the equiplet
	 * 
	 * @param message
	 */
	private void handleProductArrived(ACLMessage message) {
		try {
			Pair<Tick, Integer> information = Parser.parseProductArrived(message.getContent());
			equiplet.notifyProductArrived(message.getSender(), information.first);

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

	/**
	 * handle the request of a product agent to release the scheduled time slots
	 * 
	 * @param message
	 */
	private void handleProductRelease(ACLMessage message) {
		try {
			if (equiplet.releaseTimeSlopts(message.getSender())) {
				// send can reply
				ACLMessage reply = message.createReply();
				reply.setContent(Parser.parseConfirmation(true));
				reply.setPerformative(ACLMessage.CONFIRM);
				equiplet.send(reply);
			} else {
				ACLMessage reply = message.createReply();
				reply.setContent(Parser.parseConfirmation(false));
				reply.setPerformative(ACLMessage.DISCONFIRM);
				equiplet.send(reply);
			}
		} catch (JSONException e) {
			System.err.printf("EA:%s failed to parse product release time slots.\n", equiplet.getLocalName());
			System.err.printf("EA:%s %s", equiplet.getLocalName(), e.getMessage());
		}
	}

	/**
	 * handle the request for current information of an equiplet
	 * 
	 * @param message
	 */
	private void handleInformationRequest(ACLMessage message) {
		try {
			// TODO Auto-generated method stub
			JSONObject equipletUpdate = new JSONObject();
			equipletUpdate.put("receiver", "interface");
			equipletUpdate.put("subject", "update_equiplet");
			equipletUpdate.put("id", equiplet.getLocalName());
			equipletUpdate.put("services", equiplet.getCapabilities());

			JSONObject status = new JSONObject();
			status.put("type", "success");
			status.put("content", "NORMAL");
			equipletUpdate.put("status", status);

			JSONObject mode = new JSONObject();
			mode.put("type", "success");
			mode.put("content", "NORMAL");
			equipletUpdate.put("mode", mode);

			JSONObject equipletDetails = new JSONObject();
			equipletDetails.put("status", equiplet.getEquipletState());
			equipletDetails.put("plannedSteps", equiplet.getWaiting());
			equipletDetails.put("successfulSteps", equiplet.getExecuted());
			equipletDetails.put("failedSteps", equiplet.getExecuted());

			equipletUpdate.put("details", equipletDetails);

			// send information reply
			ACLMessage reply = message.createReply();
			reply.setPerformative(ACLMessage.INFORM);
			reply.setContent(equipletUpdate.toString());
			equiplet.send(reply);

		} catch (JSONException e) {
			System.err.println("EA: " + myAgent.getLocalName() + " something wrong with sending information update");
		}
	}
}
