package MAS.equiplet;

import generic.Mast;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import util.log.Logger;

import jade.lang.acl.ACLMessage;

/**
 * Handle get data messages
 * 
 * @author Kevin Bosman
 *
 */
public class EquipletGetDataHandler{
	private EquipletAgent equiplet;
	private HardwareAbstractionLayer hal;
	
	protected Mast.State currentMastState = Mast.State.OFFLINE;
	protected Mast.Mode currentMastMode = Mast.Mode.NORMAL;
	
	public EquipletGetDataHandler(EquipletAgent e, HardwareAbstractionLayer h){
		equiplet = e;
		hal = h;
		
		//HAL not yet used (used to avoid not used warning)
		hal.getEquipletName();
	}
	
	/**
	 * Handle get data request messages and respond appropriately 
	 * 
	 * @param ACLMessage that was received
	 * @author Kevin Bosman
	 */
	public void handleEquipletGetRequest(ACLMessage msg){
		if(msg != null){
			JSONObject result = new JSONObject();
			try{
				JSONObject messageContent = new JSONObject(msg.getContent());
				
				//Debug output
				Logger.log("Content of message: " + messageContent.toString());
				
				//Identifying requested equiplet command
				String command = messageContent.getString("command").toString();
				
				//Delegate command to corresponding functions
				switch(command){
				case "GET_CURRENT_EQUIPLET_STATE":
					result = getCurrentEquipletState();
					break;
					
				case "GET_CURRENT_MAST_MODE":
					result = getCurrentMastMode();
					break;
					
				case "GET_ALL_MODULES":
					result = getAllModules();
					break;
				
				case "GET_MODULE_STATE":
					result = getModuleState();
					break;
					
				case "GET_ALL_MODULES_STATES":
					result = getAllModulesStates();
					break;
					
				case "GET_SCHEDULE":
					result = getSchedule();
					break;
					
				case "GET_ALL_POSIBLE_STATES":
					result = getAllPosibleStates();
					break;
					
				case "GET_ALL_POSIBLE_MODES":
					result = getAllPosibleModes();
					break;
					
				default:
					Logger.log("Invalid equiplet getRequest: " + command);
					break;
				}
			}catch(Exception e){
				Logger.log("No equiplet command specified");
				e.printStackTrace();
			}
			
			ACLMessage reply = msg.createReply();
			if(result != null){
				reply.setPerformative(ACLMessage.INFORM);
				reply.setContent(result.toString());
			}else{
				reply.setPerformative(ACLMessage.FAILURE);
			}
			
			Logger.log("Get data result: " + result);
			
			equiplet.send(reply);
		}
	}
	
	/**
	 * Request current equiplet mast state
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getCurrentEquipletState(){
		JSONObject result = new JSONObject();
		try {
			result.put("state", currentMastState.toString());
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request current equiplet mast mode
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getCurrentMastMode(){
		JSONObject result = new JSONObject();
		try {
			result.put("mode", currentMastMode.toString());
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request all currently connected modules [WIP]
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getAllModules(){
		JSONObject result = new JSONObject();
		try {
			result.put("state", equiplet.state);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request state of a single module [WIP][To be discussed]
	 * 
	 * @param Module identifier?
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getModuleState(){
		JSONObject result = new JSONObject();
		try {
			result.put("state", equiplet.state);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request state of all connected modules [WIP][To be discussed]
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getAllModulesStates(){
		JSONObject result = new JSONObject();
		try {
			result.put("state", equiplet.state);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request total schedule[WIP]
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getSchedule(){
		JSONObject result = new JSONObject();
		try {
			result.put("state", equiplet.state);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Get all Mast States the equiplet can be in
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getAllPosibleStates(){
		JSONObject result = new JSONObject();
		try {
			Mast.State[] states = Mast.State.values();
			JSONArray stateStrings = new JSONArray();
			for(Mast.State state : states){
				stateStrings.put(state.toString());
			}
			result.put("states", stateStrings);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Get all Mast Modes the equiplet can be in
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 */
	public JSONObject getAllPosibleModes(){
		JSONObject result = new JSONObject();
		try {
			Mast.Mode[] modes = Mast.Mode.values();
			JSONArray modeStrings = new JSONArray();
			for(Mast.Mode state : modes){
				modeStrings.put(state.toString());
			}
			result.put("modes", modeStrings);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Set new Mast state that will be send on state request
	 * 
	 * @param new Mast.State
	 * @author Kevin Bosman
	 */
	public void setEquipletMastState(Mast.State state){
		currentMastState = state;
	}
	
	/**
	 * Set new Mast mode that will be send on mode request
	 * 
	 * @param new Mast.Mode
	 * @author Kevin Bosman
	 */
	public void setEquipletMastMode(Mast.Mode mode){
		currentMastMode = mode;
	}

}
