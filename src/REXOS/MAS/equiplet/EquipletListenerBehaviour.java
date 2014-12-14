package MAS.equiplet;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;

import java.util.List;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;

import MAS.agents.data_classes.MessageType;
import MAS.product.ProductStep;
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
		// Listen only possible incoming conversation ids, note that otherwise the simulation would jam as the listener receives messages that else where is waited upon
		// MessageTemplate template = MessageTemplate.not(MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.DISCONFIRM),
		// MessageTemplate.or(MessageTemplate.MatchPerformative(ACLMessage.CONFIRM), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_FINISHED))));
		//MessageTemplate template = MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED), MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_CAN_EXECUTE), MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_HEARTBEAT), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE))));
		MessageTemplate template = MessageTemplate.or(MessageTemplate.or(MessageTemplate.or(MessageTemplate.MatchConversationId(Ontology.CONVERSATION_PRODUCT_ARRIVED), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_CAN_EXECUTE)), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_HEARTBEAT)), MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE));
		ACLMessage msg = equiplet.blockingReceive(template);
		if (msg != null) {
			if(msg.getSender().equals("MonitoringAgent") ){
				System.out.printf("EA:%s received message [sender=%s, performative=%s, conversation=%s, content=%s, ontology=%s]\n", equiplet.getLocalName(), msg.getSender().getLocalName(), msg.getPerformative(), msg.getConversationId(), msg.getContent(), msg.getOntology());
			}

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
			case ACLMessage.FAILURE:
				handleHearthbeat(msg);
				break;
			default:
				System.out.println("EA: "+ myAgent.getLocalName() + "default");
				break;
			}
		}
	}

	private void handleHearthbeat(ACLMessage msg) {
		try {
			// TODO Auto-generated method stub
			JSONObject equipletUpdate = new JSONObject();
			equipletUpdate.put("receiver", "interface");
			equipletUpdate.put("subject", "update_equiplet");
			equipletUpdate.put("id", equiplet.getLocalName());
			equipletUpdate.put("services", "asd");
			JSONObject status = new JSONObject();
			status.put("type", "success");
			status.put("content", "NORMAL");
			equipletUpdate.put("status", status);
			
			JSONObject mode = new JSONObject();
			mode.put("type", "success");
			mode.put("content", "NORMAL");
			equipletUpdate.put("mode", mode);
	
			JSONObject equipletDetails = new JSONObject();
			equipletDetails.put("status", equiplet.getEquipletStatus());
			equipletDetails.put("plannedSteps", equiplet.getCurQueueSize());
			equipletDetails.put("successfulSteps", equiplet.getExecuted());
			equipletDetails.put("failedSteps",  equiplet.getExecuted());
			
			equipletUpdate.put("details", equipletDetails);
			
			ACLMessage reply = msg.createReply();
	        reply.setPerformative( MessageType.PULSE_UPDATE );
	        reply.setContent(equipletUpdate.toString());
	        equiplet.send(reply);
		}catch(JSONException ex){
			System.err.println("EA: " + myAgent.getLocalName() + " somethings wrong with the heartbeat");
		}
	}

	@Override
	public boolean done() {
		return done;
	}

	private void handleCanExecute(ACLMessage message) {
		try {
			// can the equiplet execute the Triple < from time, within deadline, the following product steps >
			Triple<Tick, Tick, List<ProductStep>> question = Parser.parseCanExecute(message.getContent());

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
			List<Tuple<Integer, Pair<Tick, Tick>, String, Map<String, Object>>> data = Parser.parseScheduleRequest(message.getContent());
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
			Pair<Tick, Integer> information = Parser.parseProductArrived(message.getContent());
			
			// send can reply
			ACLMessage reply = message.createReply();
			reply.setContent(Parser.parseConfirmation(true));
			reply.setPerformative(ACLMessage.CONFIRM);
			equiplet.send(reply);
			System.out.println("EA:"+this.myAgent.getLocalName() +  " Reply to onproductArrived sent to " + message.getSender());
			
			equiplet.notifyProductArrived(message.getSender(), information.first);			
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
