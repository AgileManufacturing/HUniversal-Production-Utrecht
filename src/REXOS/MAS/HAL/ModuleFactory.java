package HAL;

import java.util.ArrayList;

import com.google.gson.Gson;

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
	public boolean insertModule(ModuleIdentifier moduleIdentifier,Gson module, Gson dynamicSettings, Gson staticSettings){
		return true;
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier,Gson module, Gson dynamicSettings){
		return true;
	}
	public Gson deleteModule(ModuleIdentifier moduleIdentifier){
		return null;
	}
}
