package HAL;

import java.util.ArrayList;

import com.google.gson.JsonObject;

import HAL.listeners.ModuleListener;

public class ModuleFactory {
	public ModuleFactory(ModuleListener moduleListener){
		
	}
	public ArrayList<Module> getBottomModules(){
		return null;
	}
	public void executeHardwareStep(HardwareStep hardwareStep){
		
	}
	public Module getModuleByIdentifier(ModuleIdentifier moduleIdentifier){
		return null;
		
	}
	public boolean insertModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings, JsonObject staticSettings){
		return true;
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings){
		return true;
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier){
		return null;
	}
}
