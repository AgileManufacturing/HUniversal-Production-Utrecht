package HAL;

import java.util.ArrayList;

import com.mongodb.util.JSON;

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
	public boolean insertModule(ModuleIdentifier moduleIdentifier,JSON module, JSON dynamicSettings, JSON staticSettings){
		return true;
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier,JSON module, JSON dynamicSettings){
		return true;
	}
	public JSON deleteModule(ModuleIdentifier moduleIdentifier){
		return null;
	}
}
