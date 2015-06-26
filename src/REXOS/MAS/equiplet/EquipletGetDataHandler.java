package MAS.equiplet;

import java.util.ArrayList;

import generic.Mast;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import SCADA.JSONTypeBuilder;
import util.log.Logger;
import jade.lang.acl.ACLMessage;

/**
 * Handle get data messages
 * 
 * @author Kevin Bosman
 * @author Thomas Kok
 * @author Mitchell van Rijkom
 *
 */
public class EquipletGetDataHandler{
	private EquipletAgent equiplet;
	private HardwareAbstractionLayer hal;
	
	public EquipletGetDataHandler(EquipletAgent e, HardwareAbstractionLayer h){
		equiplet = e;
		hal = h;
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
				
				//Identifying requested equiplet command
				String command = messageContent.getString("command");
				
				//Delegate command to corresponding functions
				switch(command){
				case "GET_BASIC_INFO":
					result = getBasicInfo();
					break;
				case "GET_DETAILED_INFO":
					result = getDetailedInfo(messageContent);
			
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
					
				case "GET_ALL_MODULES_STATES":
					result = getAllModulesStates();
					break;
				
				case "GET ALL_MODULES_MODES":
					result = getAllModulesModes();
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
				result.put("command", command);
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
			result.put("state", equiplet.getCurrentState().toString());
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
			result.put("mode", equiplet.getCurrentMode().toString());
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * Request all currently connected modules with states and modes
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 * @author Mitchell van Rijkom
	 * @author Auke de Witte
	 * @author Thomas Kok
	 */
	public JSONObject getAllModules(){
		JSONObject result = new JSONObject();
		ArrayList<Module> moduleList = new ArrayList<Module>();
		moduleList = hal.getModules();		
		
		try {
			JSONArray modulesArray = new JSONArray();
			for(Module module : moduleList){
				JSONObject JSONModuleInfo = new JSONObject();
				JSONModuleInfo.put("serialNumber", module.getModuleIdentifier().serialNumber);
				JSONModuleInfo.put("typeNumber", module.getModuleIdentifier().typeNumber);
				JSONModuleInfo.put("manufacturer", module.getModuleIdentifier().manufacturer);
				JSONModuleInfo.put("name", module.getModuleIdentifier().manufacturer + " " + module.getModuleIdentifier().typeNumber + " " +  module.getModuleIdentifier().serialNumber);
				JSONModuleInfo.put("state", module.getModuleState());
				JSONModuleInfo.put("mode", module.getModuleMode());
				modulesArray.put(JSONModuleInfo);				
			}
			result.put("modules", modulesArray);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * 	Request states from all modules
	 * @return JSONObject with response
	 * @author Auke de Witte
	 */
	public JSONObject getAllModulesStates(){
		JSONObject result = new JSONObject();
		ArrayList<Module> moduleList = hal.getModules();		
		try {
			JSONArray modulesArray = new JSONArray();
			for(Module module : moduleList){
				JSONObject JSONModuleInfo = new JSONObject();
				JSONModuleInfo.put("serialNumber", module.getModuleIdentifier().serialNumber);
				JSONModuleInfo.put("typeNumber", module.getModuleIdentifier().typeNumber);
				JSONModuleInfo.put("manufacturer", module.getModuleIdentifier().manufacturer);  
				JSONModuleInfo.put("state", module.getModuleState());
				modulesArray.put(JSONModuleInfo);
			}
			result.put("modules", modulesArray);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	
	/**
	 * 	Request modes from all modules
	 * @return JSONObject with response
	 * @author Auke de Witte
	 */
	public JSONObject getAllModulesModes(){
		JSONObject result = new JSONObject();
		ArrayList<Module> moduleList = hal.getModules();		
		try {
			JSONArray modulesArray = new JSONArray();
			for(Module module : moduleList){
				JSONObject JSONModuleInfo = new JSONObject();
				JSONModuleInfo.put("serialNumber", module.getModuleIdentifier().serialNumber);
				JSONModuleInfo.put("typeNumber", module.getModuleIdentifier().typeNumber);
				JSONModuleInfo.put("manufacturer", module.getModuleIdentifier().manufacturer);  
				JSONModuleInfo.put("mode", module.getModuleMode());
				modulesArray.put(JSONModuleInfo);
			}
			result.put("modules", modulesArray);
		} catch (JSONException e) {
			Logger.log("Error");
			return null;
		}
		return result;
	}
	/**
	 * Request total schedule
	 * 
	 * @return JSONObject with response
	 * @author Kevin Bosman
	 * @author Mitchell van Rijkom
	 * @author Thomas Kok
	 */
	public JSONObject getSchedule(){
		JSONObject result = new JSONObject();
		try {
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
	 * @author Thomas Kok
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
	
	public JSONObject getBasicInfo(){
		JSONObject result = new JSONObject();
		try {
			JSONObject agent = new JSONObject();
			agent.put("id", equiplet.getAID().getLocalName());
			agent.put("type", "EquipletAgent");
			agent.put("state", equiplet.getCurrentState().name());
			result.put("agent", agent);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return null;
		}
		return result;
	}
	
	public JSONObject getDetailedInfo(JSONObject content){
		JSONObject result = new JSONObject();
		try {
			result.put("client", content.getInt("client"));
			JSONObject agent = new JSONObject();
			JSONTypeBuilder typeBuilder = new JSONTypeBuilder();
			agent.put("id", typeBuilder.getStringObject(equiplet.getAID().getLocalName(), true, true));
			agent.put("type", typeBuilder.getStringObject("EquipletAgent", true, true));
			agent.put("state", typeBuilder.getStringObject(equiplet.getCurrentState().name(), false, true));
			agent.put("mode", typeBuilder.getStringObject(equiplet.getCurrentMode().toString(), false, true));
			agent.put("schedule", this.getSchedule());
			agent.put("modules", this.getAllModules());
			result.put("agent", agent);
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return null;
		}
		return result;
	}
}
