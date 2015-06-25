package MAS.equiplet;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import generic.Mast;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.dataTypes.ModuleIdentifier;
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
		ON_MODULE_MODE_CHANGED,
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
				
				//Identifying requested equiplet command
				String requestedEquipletCommand = command.getString("command").toString();
				String requestedEquipletAction = command.getString("action").toString();
				
				boolean isSuccesfullyAdded = false;
				boolean isValidOnChangedType = false;
				OnChangedTypes type = null;
				
				//Get listener type
				for(OnChangedTypes types : OnChangedTypes.values()){
					if(types.toString().equals(requestedEquipletCommand)){
						type = types;
						isValidOnChangedType = true;
					}
				}
				
				//Create basic message
				ACLMessage reply = msg.createReply();
				JSONObject replyMessage = new JSONObject();
				try {
					replyMessage.put("Request", new JSONObject(msg.getContent()));
					replyMessage.put("command", new JSONObject(msg.getContent()).getString("command"));
					replyMessage.put("action", new JSONObject(msg.getContent()).getString("action"));
				} catch (JSONException e) {
					e.printStackTrace();
				}
				reply.setContent(replyMessage.toString());
				
				
				if(isValidOnChangedType){
					//Execute (de)-registration procedure
					switch(requestedEquipletAction){
					case "REGISTER_LISTENER":						
						isSuccesfullyAdded = registerListener(msg.getSender(),type);
						//isSuccesfullyAdded = x;
						break;					
					case "DEREGISTER_LISTENER":
						isSuccesfullyAdded = deregisterListener(msg.getSender(),type);
						break;	
					
					}
					
					//Set performative
					if(isSuccesfullyAdded){					
						reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
					}else {
						reply.setPerformative(ACLMessage.REJECT_PROPOSAL);					
					}
				}else {
					// if onChangeType is not detected 
					reply.setPerformative(ACLMessage.NOT_UNDERSTOOD);	
				}
				
				//Send message
				equiplet.send(reply);
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
					isRemovedSuccesfully = removeEquipletFromMap(sender, types);	
				}
			}
		} else {		
			isRemovedSuccesfully = removeEquipletFromMap(sender, type);			
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
		boolean result = false;
		if(!equipletListeners.containsKey(type)){
			equipletListeners.put(type, new HashSet<AID>());
			equipletListeners.get(type).add(sender);
			result = true;
		} else {
			if(!equipletListeners.get(type).contains(sender)){
				equipletListeners.get(type).add(sender);
				result = true;
			}
		}
		return result;
	}
	
	/**
	 * This function removes an equiplet in a Map 
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet wants to listen
	 * @author Mitchell van Rijkom
	 * @return true if equiplet register was succesfull else false
	 */	
	private boolean removeEquipletFromMap(AID sender, OnChangedTypes type) {
		boolean result = false;
		if(equipletListeners.containsKey(type)){
			if(equipletListeners.get(type).contains(sender)){
				equipletListeners.get(type).remove(sender);
				result = true;
			}
		}
		return result;
	}

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
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Invalid JSON:\n");
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
	public void onModuleStateChanged(ModuleIdentifier module, Mast.State state) {
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
	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mast mode changed from a certain module
	 * @param module module that changed from mast mode 
	 * @param mode mast mode
	 * @author Auke de Witte
	 */
	public void onModuleModeChanged(ModuleIdentifier module, Mast.Mode mode) {
		String modeString = mode.toString();
		String moduleString = module.toString();
		
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			returnMessage.put("command", "ON_MODULE_MODE_CHANGED");
			returnMessage.put("module", moduleString);
			returnMessage.put("moduleMode", modeString);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		OnChangedTypes type = OnChangedTypes.ON_MODULE_MODE_CHANGED;
		notifySubscribers(type, returnMessage);	
	}
	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mode is changed from the equiplet
	 * @param mode mode of equiplet
	 * @author Mitchell van Rijkom
	 */
	public void onEquipletModeChanged(Mast.Mode mode) {
		String modeString = mode.toString();
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			returnMessage.put("command", "ON_EQUIPLET_MODE_CHANGED");
			returnMessage.put("mode", modeString);
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
		message.setConversationId(Ontology.CONVERSATION_SCADA_COMMAND);
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
	 * 
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 * @author Mitchell van Rijkom
	 */

	public void notifySubscribersOnTakeDown() {
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
