package MAS.equiplet;

import jade.core.behaviours.Behaviour;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import jade.lang.acl.UnreadableException;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;

import HAL.dataTypes.ModuleIdentifier;
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
												MessageTemplate.MatchConversationId(Ontology.CONVERSATION_SCHEDULE), 
													MessageTemplate.MatchConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND))))), 
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
				handleInformationRequest(msg);
				break;
			// messagetype holding the requested state for the equiplet
			case ACLMessage.PROPOSE:
				if(msg.getConversationId().equals(Ontology.CONVERSATION_EQUIPLET_COMMAND)){
					handleEquipletCommand(msg);
				}
				break;
			default:
				break;
			}
		}
	}

	private void handleEquipletCommand(ACLMessage msg) {
		if(msg != null){
			Serializable content = null;
			try{
				content = msg.getContentObject();
			}catch(UnreadableException uex){
				Logger.log("An error occured while attempting to getContentObject from the ACLMessage. Message will not be handled.");
			}
			if(content != null){
				JSONObject modulesInJSON = new JSONObject(content);
				String requestedEquipletCommand = "";
				ArrayList<ModuleIdentifier> modules = deserializeACLMessage(modulesInJSON, requestedEquipletCommand);
				
				// Program if statements that will appropriately handle messages sent to the equiplet agent.
				if(requestedEquipletCommand == "RECONFIGURE" && modules != null){
					equiplet.reconfigureEquiplet(modules);
				}else{
					Logger.log("An error occured while deserializing the ACLMessage, missing info or command not recognized.");
				}
			}
		}		
	}
	
	/**
	 * Dedicated function to translate the reconfigure ACLMessage in JSON format received from scada.
	 * 
	 * @param content
	 * @return Function returns null if anything went wrong while deserializing. If not, it returns an ArrayList of ModuleIdentifiers.
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 * @author Mitchell van Rijkom
	 */

	private ArrayList<ModuleIdentifier> deserializeACLMessage(JSONObject content, String requestedEquipletCommand){
		ArrayList<ModuleIdentifier> resultArray = new ArrayList<ModuleIdentifier>();
		JSONArray modules = null;
		boolean isDeserializationSuccessfull = true;
		try{
			modules = content.getJSONArray("modules");
			requestedEquipletCommand = content.getString("requested-equiplet-command");
			
			for(int i = 0; i < modules.length(); i++){
				JSONArray moduleIdentifiers = null;
				moduleIdentifiers = modules.getJSONArray(i);
				resultArray.add(new ModuleIdentifier(moduleIdentifiers.getString(0), moduleIdentifiers.getString(1), moduleIdentifiers.getString(2)));
			}
		}catch(JSONException ex){
			Logger.log("An error occured while attempting to get information from the JSON.");
			isDeserializationSuccessfull = false;
		}
		// If something went wrong while deserializing, return null.
		return isDeserializationSuccessfull ? resultArray : null;
	}
	
	/**
	 * Dedicated function to translate the reconfigure ACLMessage in String format received from scada.
	 * 
	 * @param content
	 * @return Function returns null if anything went wrong while delimiting. Otherwise it returns an ArrayList of ModuleIdentifiers.
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	
	private ArrayList<ModuleIdentifier> delimitACLMessage(String content) {
		String[] modules = content.split(";");
		ArrayList<ModuleIdentifier> resultArray = new ArrayList<ModuleIdentifier>();
		boolean isSuccessfullyDelimited = true;
		for (String module : modules){
			String[] identifiers = module.split(",");
			if((identifiers[0] != null) && (identifiers[1] != null) && (identifiers[2] != null)){
				resultArray.add(new ModuleIdentifier(identifiers[0], identifiers[1], identifiers[2]));
			}else{
				isSuccessfullyDelimited = false;
			}
		}
		// Returns null if an error occured or if information was incomplete. 
		return isSuccessfullyDelimited ? resultArray : null;
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

			// send can reply enzo
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
