package MAS.equiplet;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import generic.Mast;

import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import MAS.util.Ontology;
import jade.core.AID;
import jade.lang.acl.ACLMessage;

/**
 * Handle reconfigure steps
 * 
 * @author Kevin Bosman
 * @author Thomas Kok
 * @author Mitchell van Rijkom
 *
 */
public class EquipletOnChangedHandler{
	
	public enum OnChangedTypes {
		ALL,
		ON_EQUIPLET_STATE_CHANGED,
		ON_SCHEDULE_CHANGED,
		ON_MODULE_STATE_CHANGED,
		ON_EQUIPLET_MODE_CHANGED
	}
	private Map<OnChangedTypes, Set<AID>> equipletListeners;
	
	private EquipletAgent equiplet;
	
	public EquipletOnChangedHandler(EquipletAgent e, HardwareAbstractionLayer h){
		equiplet = e;
		equipletListeners = new HashMap<OnChangedTypes, Set<AID>>();
		
		//This print line is because that autistic neighbor of mine can't handle an unused warning...
		System.out.println(equiplet.state);
	}
	
	/**
	 * Handles ACLMessage if an equiplet want to register to an certain state changed 
	 *  
	 * @param msg ACLMessage 
	 * @author Mitchell van Rijkom
	 * @author Kevin Bosman
	 */
	public void handleOnChangedListenerCommand(ACLMessage msg){
		if(msg != null){
			try{
				JSONObject command = new JSONObject(msg.getContent());
				
				//Debug output
				//Logger.log("Content of message: " + command.toString());
				
				//Identifying requested equiplet command
				String requestedEquipletCommand = command.getString("command").toString();
				String requestedEquipletAction = command.getString("action").toString();
				
				//Logger.log("command = " + requestedEquipletCommand + " action = " + requestedEquipletAction);
				
				boolean isSuccesfullyAdded = false;
				boolean isValidOnChangedType = false;
				OnChangedTypes type = null;
				
				// test to loop through enum and print values
//				for(OnChangedTypes types : OnChangedTypes.values()){
//					Logger.log("OnChangedTypes: " + types.toString());
//				}
				
				//Get listener type
				for(OnChangedTypes types : OnChangedTypes.values()){
					if(types.toString().equals(requestedEquipletCommand)){
						type = types;
						isValidOnChangedType = true;
						//Logger.log("Type match with " + type.toString());
					}
				}							
				
				if(isValidOnChangedType){
					//Logger.log("Execute (de)-registration procedure");	
					//Execute (de)-registration procedure
					switch(requestedEquipletAction){
					case "REGISTER_LISTENER":						
						isSuccesfullyAdded = registerListener(msg.getSender(),type);
						//Logger.log("case REGISTER_LISTENER == " + isSuccesfullyAdded);	
						//isSuccesfullyAdded = x;
						break;					
					case "DEREGISTER_LISTENER":
						//Logger.log("case REGISTER_LISTENER");	
						isSuccesfullyAdded = deregisterListener(msg.getSender(),type);
						break;	
					
					}
					
					//Send reply
					ACLMessage reply = msg.createReply();
					JSONObject replyMessage = new JSONObject();
					try {
						replyMessage.put("Request", new JSONObject(msg.getContent()));
					} catch (JSONException e) {
						e.printStackTrace();
					}
					reply.setContent(replyMessage.toString());
					
					if(isSuccesfullyAdded){					
						reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
					}else {
						reply.setPerformative(ACLMessage.REJECT_PROPOSAL);					
					}
					equiplet.send(reply);
				}else {
					// if onChangeType is not detected 
					ACLMessage reply = msg.createReply();
					JSONObject replyMessage = new JSONObject();
					try {
						replyMessage.put("Request", new JSONObject(msg.getContent()));
					} catch (JSONException e) {
						e.printStackTrace();
					}
					reply.setContent(replyMessage.toString());
					reply.setPerformative(ACLMessage.NOT_UNDERSTOOD);	
					equiplet.send(reply);
				}
			}catch(Exception e){
				
			}
		}
	}

	
	/**
	 * This function register an equiplet that will by notified when an state changed is called
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet wants to listen
	 * @author Mitchell van Rijkom
	 * @return true if equiplet register was succesfull else false
	 */	
	private boolean registerListener(AID sender, OnChangedTypes type) {
		//Logger.log("In function registerListener");
		
		boolean isAddedSuccesfully = false;
		if(type.equals(OnChangedTypes.ALL)){
			for(OnChangedTypes types : OnChangedTypes.values()){
				if(!types.equals(OnChangedTypes.ALL)){
					isAddedSuccesfully = addEquipletToMap(sender, types);	
				}
			}
		} else {		
			isAddedSuccesfully = addEquipletToMap(sender, type);			
		}
		
		
		return isAddedSuccesfully;			
	}
	
	/**
	 * This function deregister an equiplet that will by notified when an state changed is called
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet not wants to listen anymore
	 * @author Mitchell van Rijkom
	 * @return true if equiplet deregister was succesfull else false
	 */	
	private boolean deregisterListener(AID sender, OnChangedTypes type) {
		boolean isRemovedSuccesfully = false;
		if(type.equals(OnChangedTypes.ALL)){
			for(OnChangedTypes types : OnChangedTypes.values()){
				if(!types.equals(OnChangedTypes.ALL)){
					isRemovedSuccesfully = removeEquipletToMap(sender, types);	
				}
			}
		} else {		
			isRemovedSuccesfully = removeEquipletToMap(sender, type);			
		}
		
		return isRemovedSuccesfully;		
	}

	/**
	 * This function adds an equiplet in a Map 
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet wants to listen
	 * @author Mitchell van Rijkom
	 * @return true if equiplet register was succesfull else false
	 */	
	private boolean addEquipletToMap(AID sender, OnChangedTypes type) {
		if(!equipletListeners.containsKey(type)){
			equipletListeners.put(type, new HashSet<AID>());
			equipletListeners.get(type).add(sender);
			//Logger.log("Succesfully registerd " + sender.toString() + "with type " + type.toString());
			return true;
		} else {
			if(!equipletListeners.get(type).contains(sender)){
				equipletListeners.get(type).add(sender);
				//Logger.log("Succesfully registerd " + sender.toString() + "with type " + type.toString());
				return true;
			}
		}
		//Logger.log("Failed deregister " + sender.toString() + "with type " + type.toString());
		return false;
	}
	
	/**
	 * This function removes an equiplet in a Map 
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet wants to listen
	 * @author Mitchell van Rijkom
	 * @return true if equiplet register was succesfull else false
	 */	
	private boolean removeEquipletToMap(AID sender, OnChangedTypes type) {
		if(equipletListeners.containsKey(type)){
			if(equipletListeners.get(type).contains(sender)){
				equipletListeners.get(type).remove(sender);
				//Logger.log("Succesfully deregisterd " + sender.toString() + "with type " + type.toString());
				return true;
			}else {
				//Logger.log("Failed deregister " + sender.toString() + "with type " + type.toString());
				return false;
			}
		} else {
			//Logger.log("Failed deregister " + sender.toString() + "with type " + type.toString());
			return false;
		}
	}

	/**
	 * Test function to log all the equiplets that are registered by an onChangeType
	 * @param type onChangeType
	 * @author Mitchell van Rijkom
	 */
	/*
	private void logEquipletsByType(OnChangedTypes type) {
		// List al equiplets to test
		for(Map.Entry<OnChangedTypes, Set<AID>> entry : equipletListeners.entrySet()){
			if(entry.getKey() == type){
				for(AID senderID : entry.getValue()){
					Logger.log("Type: " + entry.toString() +  "    Equiplet: " + senderID);
				}
			}
		}
	}
	*/
	
	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mast state changed 
	 * @param state mast state
	 * @author Mitchell van Rijkom
	 */
	public void onEquipletStateChanged(Mast.State state){	
		System.out.println("EQ: change state");
		String stateString = state.toString();
		
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			JSONObject agent = new JSONObject();
			agent.put("id", equiplet.getLocalName());
			agent.put("state", stateString);
			returnMessage.put("agent", agent);
			returnMessage.put("command", "ON_EQUIPLET_STATE_CHANGED");
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		OnChangedTypes type = OnChangedTypes.ON_EQUIPLET_STATE_CHANGED;
		notifySubscribers(type, returnMessage);	
	}	
	
	public void onScheduleChanged(JSONObject command) {
		//TODO Make implementation
	}
	
	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mast state changed from a certain module
	 * @param module module that changed from mast state 
	 * @param state mast state
	 * @author Mitchell van Rijkom
	 */
	public void onModuleStateChanged(Module module, Mast.State state) {
		String stateString = state.toString();
		String moduleString = module.toString();
		
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			returnMessage.put("command", "ON_MODULE_STATE_CHANGED");
			returnMessage.put("module", moduleString);
			returnMessage.put("moduleState", stateString);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		OnChangedTypes type = OnChangedTypes.ON_MODULE_STATE_CHANGED;
		notifySubscribers(type, returnMessage);	
	}
	 /* @param returnMessage onChange message 
	 * @author Mitchell van Rijkom

	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mode is changed from the equiplet
	 * @param mode mode of equiplet
	 * @author Mitchell van Rijkom
	 */
	public void onEquipletModeChanged(Mast.Mode mode) {
		String modeString = mode.toString();
		// statesArray is not yet implemented in the equiplet Agent
		// These value are dummy values in order to adhere the structure
		// from SCADA on the wiki
		String[] statesArray = {"state1", "state2", "state3"};
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			returnMessage.put("command", "ON_EQUIPLET_MODE_CHANGED");
			returnMessage.put("mode", modeString);
			returnMessage.put("possibleStates", statesArray);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		OnChangedTypes type = OnChangedTypes.ON_EQUIPLET_MODE_CHANGED;
		notifySubscribers(type, returnMessage);	
	}
	
	/**
	 * This function notifies all subsribers with an onChangeType with a returnMessage
	 * This message will send an ACLMessage to all the subcribers
	 * @param type on onChangeType
	 * @param returnMessage onChange message 
	 * @author Mitchell van Rijkom
	 * @author Kevin Bosman
	 */
	private void notifySubscribers(OnChangedTypes type, JSONObject returnMessage){
		// send message
		ACLMessage message = new ACLMessage(ACLMessage.INFORM);
		message.setContent(returnMessage.toString());
		message.setConversationId(Ontology.GRID_ONTOLOGY);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		
		// sends message to equiplets with define type
		for(Map.Entry<OnChangedTypes, Set<AID>> entry : equipletListeners.entrySet()){
			if(entry.getKey() == type){
				for(AID sender : entry.getValue()){
					message.addReceiver(sender);
				}
			}
		}		
		equiplet.send(message);	
	}
	
	/**
	 * updateSubscribersOnTakeDown() : Creates an ACLMessage containing unique AID's for agents subscribed to this agent when it shuts down,
	 * notifying them about its shutdown.
	 */

	public void updateSubscribersOnTakeDown() {
		ACLMessage takeDownMessage = new ACLMessage(ACLMessage.INFORM);
		takeDownMessage.setConversationId(Ontology.CONVERSATION_AGENT_TAKEDOWN);
		Set<AID> agentsList = new HashSet<AID>();
		JSONObject agent = new JSONObject();
		JSONObject takeDownMessageContent = new JSONObject();
		
		// Get all unique subscribed receivers to get a message of this equiplet being shutdown
		for(Map.Entry<OnChangedTypes, Set<AID>> entry : equipletListeners.entrySet()){
			for(AID receiver : entry.getValue()){
				if(!agentsList.contains(receiver)){
					agentsList.add(receiver);
				}
			}
		}
		
		// Add all the unique receivers to the message
		for(AID receiver : agentsList){
			takeDownMessage.addReceiver(receiver);
		}
		
		// Create content for the message
		try{
			agent.put("id", equiplet.getLocalName());
			agent.put("state", JSONObject.NULL);
			takeDownMessageContent.put("agent", agent);
			takeDownMessageContent.put("command", "ON_EQUIPLET_TAKEDOWN");
		}catch(JSONException ex){ex.printStackTrace();}
		
		takeDownMessage.setContent(takeDownMessageContent.toString());
		
		// Send the message
		takeDownMessage.setContent(takeDownMessageContent.toString());
		equiplet.send(takeDownMessage);
	}
}
