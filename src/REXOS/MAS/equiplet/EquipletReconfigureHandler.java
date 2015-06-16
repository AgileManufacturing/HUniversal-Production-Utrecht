package MAS.equiplet;

import java.util.ArrayList;

import generic.Mast;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.StaticSettings;

import util.log.Logger;

import jade.lang.acl.ACLMessage;

/**
 * Handle reconfigure steps
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
		
		//This print line is because that autistic neighbor of mine can't handle an unused warning...
		System.out.println(equiplet.state);
	}
	
	public void handleEquipletCommand(ACLMessage msg){
		if(msg != null){
			try{
				JSONObject command = new JSONObject(msg.getContent());
				
				//Debug output
				Logger.log("Content of message: " + command.toString());
				
				//Identifying requested equiplet command
				String requestedEquipletCommand = command.getString("command").toString();
				
				//Delegate command to corresponding functions
				switch(requestedEquipletCommand){
				case "CHANGE_EQUIPLET_MACHINE_STATE":
					changeEquipletMachineState(command);
					break;
					
				case "INSERT_MODULE":
					insertModule(command);
					break;
					
				case "DELETE_MODULE":
					deleteModule(command);
					break;
				}
			}catch(Exception e){
				
			}
		}
	}
	
	public void changeEquipletMachineState(JSONObject command){
		try {
			//Get new state
			String stateString = command.getString("state").toString();
			Mast.State state = Mast.State.valueOf(stateString);
			
			//Execute action
			hal.changeState(state);
		} catch (JSONException e) {
			//TODO error handling
			e.printStackTrace();
		}
	}
	
	public void insertModule(JSONObject command){
		try{
	//		hal.getState()???? where is this function??;
	//		if(currentState != Mast.State.SAFE){
				//Error out
	//		}
			System.out.println("Insert module func");
			
			//Deserialize modules
			JSONArray modules = (JSONArray) command.get("modules");
			deserializeModules(modules);
		} catch (JSONException e){
			//TODO error handling
			e.printStackTrace();
		}
	}
	
	public void deleteModule(JSONObject command){
		try{
			
			//Deserialize modules
			JSONArray modules = (JSONArray) command.get("modules");
			deserializeModules(modules);
		} catch (JSONException e){
			//TODO error handling
			e.printStackTrace();
		}
	}
	
	public void deserializeModules(JSONArray moduleArray){
		System.out.println(moduleArray.toString());
		/*
		ArrayList<ModuleIdentifier> modules = extractModulesForReconfig(moduleArray);	
		ArrayList<StaticSettings> staticSettings = new ArrayList<StaticSettings>();
		ArrayList<DTOModuleSettings> dtoSettings = new ArrayList<DTOModuleSettings>();
		KnowledgeDBClient kdb = new KnowledgeDBClient();
		
		//Get the staticSettings and push into array
		for(int i = 0; i < modules.size(); i++){
			//Get static settings from Grid Knowledge DB
			staticSettings.add(StaticSettings.getStaticSettingsForModuleIdentifier(modules.get(i), kdb));
			
			DTOModuleSettings dto = new DTOModuleSettings();
			//Create DTO staticSettings
			dto.staticSettings = staticSettings.get(i).serialize();
			
			//Create DTO dynamicSettings
			dto.dynamicSettings.put(DynamicSettings.ATTACHED_TO, JSONObject.NULL);
			dto.dynamicSettings.put(DynamicSettings.MOUNT_POINT_X, 1);
			dto.dynamicSettings.put(DynamicSettings.MOUNT_POINT_Y, 1);
			
			//Add DTO to array
			dtoSettings.add(dto);
			
		}*/
	}

}
