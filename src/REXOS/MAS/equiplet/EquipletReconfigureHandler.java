package MAS.equiplet;

import java.text.ParseException;
import java.util.ArrayList;

import generic.Mast;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.DynamicSettings;
import HAL.dataTypes.StaticSettings;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;

import jade.lang.acl.ACLMessage;

/**
 * Handle reconfigure messages
 * 
 * @author Kevin Bosman
 * @author Thomas Kok
 *
 */
public class EquipletReconfigureHandler{
	private EquipletAgent equiplet;
	private HardwareAbstractionLayer hal;
	
	public EquipletReconfigureHandler(EquipletAgent e, HardwareAbstractionLayer h){
		equiplet = e;
		hal = h;
	}
	
	/**
	 * Handle command messages
	 * 
	 * @param the message
	 * @return 
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	public void handleEquipletCommand(ACLMessage msg){
		if(msg != null){
			boolean succes = false;
			try{
				JSONObject messageContent = new JSONObject(msg.getContent());

				//Identifying requested equiplet command
				String command = messageContent.getString("command").toString();
				
				//Delegate command to corresponding functions
				switch(command){
				case "CHANGE_EQUIPLET_MACHINE_STATE":
					succes = changeEquipletMachineState(messageContent);
					break;
					
				case "INSERT_MODULE":
					succes = insertModule(messageContent);
					break;
					
				case "DELETE_MODULE":
					succes = deleteModule(messageContent);
					break;
					
				default:
					Logger.log("Invalid equiplet command: " + command);
					break;
				}
			}catch(Exception e){
				Logger.log("No equiplet command specified");
				e.printStackTrace();
			}
			
			//Send response
			ACLMessage reply = msg.createReply();
			JSONObject replyMessage = new JSONObject();
			try {
				replyMessage.put("Request", new JSONObject(msg.getContent()));
				replyMessage.put("command", new JSONObject(msg.getContent()).getString("command"));
			} catch (JSONException e) {
				e.printStackTrace();
			}
			reply.setContent(replyMessage.toString());
			if(succes){
				reply.setPerformative(ACLMessage.CONFIRM);
			}else{
				reply.setPerformative(ACLMessage.FAILURE);
			}
			equiplet.send(reply);
		}
	}
	
	/**
	 * Change the Mast state of the equiplet
	 * 
	 * @param content
	 * @return succes
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	public boolean changeEquipletMachineState(JSONObject messageContent){
		try {
			//Get new state
			String stateString = messageContent.getString("state").toString();
			Mast.State state = Mast.State.valueOf(stateString);
			
			if(state != null){
				//Execute action
				hal.changeState(state);
				return true;
			}else{
				//Error message
				Logger.log("Invalid Mast State: " + stateString);
			}
		} catch (JSONException e) {
			Logger.log("No Mast State specified");
		}
		return false;
	}
	
	/**
	 * Function to insert modules
	 * 
	 * @param content
	 * @return succes
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	public boolean insertModule(JSONObject messageContent){
		boolean result = false;
		try{
			if(equiplet.getCurrentState() == Mast.State.SAFE || equiplet.getCurrentState() == Mast.State.OFFLINE){
				
				//Deserialize modules
				JSONArray modules = (JSONArray) messageContent.get("modules");
				ArrayList<ModuleIdentifier> moduleIdentifiers = deserializeModules(modules);
				
				//Get static and dynamic settings
				ArrayList<JSONObject> staticSettings = new ArrayList<JSONObject>();
				ArrayList<JSONObject> dynamicSettings = new ArrayList<JSONObject>();
				KnowledgeDBClient kdb = new KnowledgeDBClient();
				
				//Get the staticSettings and push into array
				for(int i = 0; i < moduleIdentifiers.size(); i++){
					//Get static settings from Grid Knowledge DB
					StaticSettings staticSetting = StaticSettings.getStaticSettingsForModuleIdentifier(moduleIdentifiers.get(i), kdb);
					if(!staticSettings.isEmpty()){
						JSONObject staticSettingJSON = staticSetting.serialize();
						staticSettings.add(staticSettingJSON);
					
						//Create dynamicSetting JSONObject
						JSONObject dynamicSetting = new JSONObject();
						dynamicSetting.put(DynamicSettings.ATTACHED_TO, messageContent.get("attached-to").toString());
						dynamicSetting.put(DynamicSettings.MOUNT_POINT_X, messageContent.get("mount-point-x").toString());
						dynamicSetting.put(DynamicSettings.MOUNT_POINT_Y, messageContent.get("mount-point-y").toString());
						dynamicSettings.add(dynamicSetting);
					}else{
						Logger.log("Settings could not be retrieved from the KnowledgeDatabase");
					}
				}
				
				//Insert module in hal
				if(moduleIdentifiers != null && staticSettings != null && moduleIdentifiers.size() == staticSettings.size()){
					for(int i = 0; i < staticSettings.size(); i++){
						hal.insertModule(staticSettings.get(i), dynamicSettings.get(i));
					}
					result = true;
				}else{
					Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Error while extracting modules for reconfiguration");
				}
			} else {
				Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.WARNING, "Equiplet must be in SAFE or OFFLINE state for reconfiguration");
			}
		} catch (JSONException e){
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Error while parsing JSON argument");
		} catch (ParseException e) {
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Error while extracting modules for reconfiguration");
		} catch (InvalidMastModeException e) {
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "InsertModule cannot be executed in current MAST mode: " + equiplet.getCurrentState());
		}
		return result;
	}
	
	/**
	 * Function to extract modules and remove them from HAL
	 * 
	 * @param content
	 * @return succes
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	public boolean deleteModule(JSONObject messageContent){
		try{
			if(equiplet.getCurrentState() == Mast.State.SAFE || equiplet.getCurrentState() == Mast.State.OFFLINE){
				//Deserialize modules
				JSONArray modules = (JSONArray) messageContent.get("modules");
				ArrayList<ModuleIdentifier> moduleIdentifiers = deserializeModules(modules);
				for(ModuleIdentifier mi: moduleIdentifiers){
					hal.deleteModule(mi);
				}
				return true;
			}else{
				Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.WARNING, "Equiplet must be in SAFE or OFFLINE state for reconfiguration");
			}
		} catch (JSONException e){
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Error occured while parsing JSON containing information on the module to be deleted.");
		} catch (Exception e) {
			Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "An error occured while attempting to delete a module.");
		}
		return false;
	}
	
	/**
	 * Dedicated function to translate the reconfigure ACLMessage in JSON format received from scada.
	 * 
	 * @param content
	 * @return Function returns null if anything went wrong while deserializing. If not, it returns an ArrayList of ModuleIdentifiers.
	 * @author Kevin Bosman
	 * @author Thomas Kok
	 */
	private ArrayList<ModuleIdentifier> deserializeModules(JSONArray modules){
		ArrayList<ModuleIdentifier> resultArray = new ArrayList<ModuleIdentifier>();
		boolean isDeserializationSuccessfull = true;
		try{
			JSONObject currentModule;
			//Loop trough the array with modules
			for(int i = 0; i < modules.length(); i++){
				//Extract each data object from array
				currentModule = modules.getJSONObject(i);
				
				//Extract module data and get a module ID to return 
				resultArray.add(new ModuleIdentifier(
					currentModule.getString("manufacturer"), 
					currentModule.getString("typeNumber"), 
					currentModule.getString("serialNumber")
				));
			}
		}catch(JSONException ex){
			Logger.log("An error occured while attempting to get information from the JSON. \n" + ex.getMessage());
			isDeserializationSuccessfull = false;
		}
		// If something went wrong while deserializing, return null.
		return isDeserializationSuccessfull ? resultArray : null;
	}
}
