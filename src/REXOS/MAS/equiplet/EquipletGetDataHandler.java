package MAS.equiplet;

import generic.Mast;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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
	
	protected Mast.State currentMastState = Mast.State.OFFLINE;
	protected Mast.Mode currentMastMode = Mast.Mode.NORMAL;
	
	public EquipletGetDataHandler(EquipletAgent e){
		equiplet = e;
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
				result.put("command", messageContent.getString("command"));
				//Debug output
				//Logger.log("Content of message: " + messageContent.toString());
				
				//Identifying requested equiplet command
				String command = messageContent.getString("command").toString();
				
				//Delegate command to corresponding functions
				switch(command){
				case "GET_BASIC_INFO":
					result.put("id", equiplet.getAID().getLocalName());
					result.put("type", "EquipletAgent");
					result.put("state", equiplet.getEquipletState().name());
					break;
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
				try {
					result.put("Request", new JSONObject(msg.getContent()));
				} catch (JSONException e) {
					e.printStackTrace();
				}
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
			result.put("command", "GET_CURRENT_MAST_MODE");
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
	 * @author Mitchell van Rijkom
	 */
	public JSONObject getSchedule(){
		JSONObject result = new JSONObject();
		try {
			result.put("command", "GET_SCHEDULE");
			JSONArray JSONSchedule = new JSONArray();
			for (Job job : equiplet.schedule) {
				Logger.log(job.toString());
				JSONObject jobData = new JSONObject();
				try {
					jobData.put("product", job.getProductAgentName() 	!= null ? job.getProductAgentName().toString() 	: "null");
					jobData.put("Name", job.getProductAgent()			!= null ? job.getProductAgent().toString() 		: "null" );
					jobData.put("index", job.getIndex());
					jobData.put("service", job.getService() 			!= null ? job.getService().toString() 			: "null" );
					jobData.put("criteria", job.getCriteria()			!= null ? job.getCriteria().toString() 			: "null" );
					jobData.put("start", job.getStartTime()				!= null ? job.getStartTime().toString() 		: "null" );
					jobData.put("due", job.getDue()						!= null ? job.getDue().toString() 				: "null" );
					jobData.put("deadline", job.getDeadline() 			!= null ? job.getDeadline().toString() 			: "null" );
					jobData.put("ready", job.getDuration() 				!= null ? job.getDuration().toString() 			: "null" );
				} catch(JSONException e){
					Logger.log("Error");
					return null;
				}
				
				JSONSchedule.put(jobData);
				
			}
			result.put("schedule", JSONSchedule);
			
			// loop through tree set and create json object with all data 
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
			result.put("command", "GET_ALL_POSIBLE_MODES");
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
	 * Set new Mast state that will be sent on state request
	 * 
	 * @param new Mast.State
	 * @author Kevin Bosman
	 */
	public void setEquipletMastState(Mast.State state){
		currentMastState = state;
	}
	
	/**
	 * Set new Mast mode that will be sent on mode request
	 * 
	 * @param new Mast.Mode
	 * @author Kevin Bosman
	 */
	public void setEquipletMastMode(Mast.Mode mode){
		currentMastMode = mode;
	}

}
