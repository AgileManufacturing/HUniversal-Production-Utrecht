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
				
				//Debug output
				Logger.log("Content of message: " + messageContent.toString());
				
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
		try{
	//		hal.getState()???? where is this function??;
	//		if(currentState != Mast.State.SAFE){
				//Error out
	//		}
			System.out.println("Insert module func");
			
			//Deserialize modules
			JSONArray modules = (JSONArray) messageContent.get("modules");
			ArrayList<ModuleIdentifier> moduleIdentifiers = deserializeModules(modules);
			
			System.out.println(moduleIdentifiers.toString());
			
			
			//Get static and dynamic settings
			ArrayList<JSONObject> staticSettings = new ArrayList<JSONObject>();
			ArrayList<JSONObject> dynamicSettings = new ArrayList<JSONObject>();
			KnowledgeDBClient kdb = new KnowledgeDBClient();
			
			//TODO generate JSONObjects for both settings and insert them into hal
			
			//Get the staticSettings and push into array
			for(int i = 0; i < moduleIdentifiers.size(); i++){
				//Get static settings from Grid Knowledge DB
				StaticSettings staticSetting = StaticSettings.getStaticSettingsForModuleIdentifier(moduleIdentifiers.get(i), kdb);
				JSONObject staticSettingJSON = staticSetting.serialize();
				staticSettings.add(staticSettingJSON);
				
				//Create dynamicSetting JSONObject
				//TODO Get actual dynamic values from JSON
				JSONObject dynamicSetting = new JSONObject();
				dynamicSetting.put(DynamicSettings.ATTACHED_TO, JSONObject.NULL);
				dynamicSetting.put(DynamicSettings.MOUNT_POINT_X, 1);
				dynamicSetting.put(DynamicSettings.MOUNT_POINT_Y, 1);
				dynamicSettings.add(dynamicSetting);
			}
			
			//Insert module in hal
			if(moduleIdentifiers != null && staticSettings != null && moduleIdentifiers.size() == staticSettings.size()){
				for(int i = 0; i < staticSettings.size(); i++){
					hal.insertModule(staticSettings.get(i), dynamicSettings.get(i));
				}
				return true;
			}else{
				Logger.log(LogSection.MAS_EQUIPLET_AGENT, LogLevel.ERROR, "Error while extracting modules for reconfiguration");
			}
		} catch (JSONException e){
			//TODO error handling
			e.printStackTrace();
		} catch (ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InvalidMastModeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return false;
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
			//TODO Check for good mast state
			
			//Deserialize modules
			JSONArray modules = (JSONArray) messageContent.get("modules");
			ArrayList<ModuleIdentifier> moduleIdentifiers = deserializeModules(modules);
			for(ModuleIdentifier mi: moduleIdentifiers){
				hal.deleteModule(mi);
			}
			return true;
		} catch (JSONException e){
			//TODO error handling
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
