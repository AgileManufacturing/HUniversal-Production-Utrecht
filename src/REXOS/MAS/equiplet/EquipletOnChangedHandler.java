package MAS.equiplet;

import java.util.Map;

import generic.Mast;
import generic.Mast.Mode;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import MAS.util.Ontology;

import util.log.Logger;

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
		ON_EQUIPLET_STATE_CHANGED,
		ON_SCHEDULE_CHANGED,
		ON_MODULE_STATE_CHANGED,
		ON_EQUIPLET_MODE_CHANGED
	}
	private Map<OnChangedTypes, AID> equipletListeners;
	
	private EquipletAgent equiplet;
	private HardwareAbstractionLayer hal;
	
	
	public EquipletOnChangedHandler(EquipletAgent e, HardwareAbstractionLayer h){
		equiplet = e;
		hal = h;
		
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
				Logger.log("Content of message: " + command.toString());
				
				//Identifying requested equiplet command
				String requestedEquipletCommand = command.getString("command").toString();
				String requestedEquipletAction = command.getString("action").toString();
				
				boolean isSuccesfullyAdded = false;
				OnChangedTypes type = null;
				
				//Get listener type
				for(OnChangedTypes types : OnChangedTypes.values()){
					if(types.toString() == requestedEquipletCommand){
						type = types;
					}
				}
								
				//Execute (de)-registration procedure
				switch(requestedEquipletAction){
				case "REGISTER_LISTENER":
					//Execute register and set succes var
					isSuccesfullyAdded = registerListener(msg.getSender(),type);
					//isSuccesfullyAdded = x;
					break;					
				case "DEREGISTER_LISTENER":
					//Execute deregister and set succes var
					isSuccesfullyAdded = deregisterListener(msg.getSender(),type);
					break;	
				
				}
				
				//Send reply
				ACLMessage reply = msg.createReply();
				if(isSuccesfullyAdded){					
					reply.setPerformative(ACLMessage.ACCEPT_PROPOSAL);
				}else {
					reply.setPerformative(ACLMessage.REJECT_PROPOSAL);					
				}
				equiplet.send(reply);
			}catch(Exception e){
				
			}
		}
	}
	
	/**
	 * This function register an equiplet in a Map that will by notified when an state changed is called
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet wants to listen
	 * @author Mitchell van Rijkom
	 * @return true if equiplet register was succesfull else false
	 */	
	private boolean registerListener(AID sender, OnChangedTypes type) {
		if(!equipletListeners.containsKey(type) && !equipletListeners.containsValue(sender)){
			equipletListeners.put(type, sender);
			return true;
		}
		return false;
	}
	
	/**
	 * This function deregister an equiplet in a Map that will by notified when an state changed is called
	 * @param sender an equiplet
	 * @param type an onChangeType where an equiplet not wants to listen anymore
	 * @author Mitchell van Rijkom
	 * @return true if equiplet deregister was succesfull else false
	 */	
	private boolean deregisterListener(AID sender, OnChangedTypes type) {
		if(equipletListeners.containsKey(type) && equipletListeners.containsValue(sender)){
			equipletListeners.remove(sender);
			return true;
		}
		return false;
	}
	
	
	/**
	 * This function notifies all equiplets that are registered to an onChangeType when a mast state changed 
	 * @param state mast state
	 * @author Mitchell van Rijkom
	 */
	public void onEquipletStateChanged(Mast.State state){		
		String stateString = state.toString();
		
		JSONObject returnMessage = new JSONObject();
		
		// create message for listeners
		try {
			returnMessage.put("command", "ON_EQUIPLET_STATE_CHANGED");
			returnMessage.put("state", stateString);
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
		message.setOntology(Ontology.GRID_ONTOLOGY);
		
		// sends message to equiplets with define type
		for(Map.Entry<OnChangedTypes, AID> entry : equipletListeners.entrySet()){
			if(entry.getKey() == type){
				message.addReceiver(entry.getValue());
			}
		}		
		equiplet.send(message);	
	}
}
