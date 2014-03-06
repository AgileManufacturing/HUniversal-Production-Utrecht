package HAL;

import java.io.Serializable;

import com.google.gson.JsonObject;

public class HardwareStep implements Serializable {
	private static final long serialVersionUID = 3031019032556167945L;
	private CompositeStep compositeStep;
	private JsonObject command;
	private ModuleIdentifier moduleIdentifier;
	
	public HardwareStep(CompositeStep compositeStep, JsonObject command, ModuleIdentifier moduleIdentifier){
		this.moduleIdentifier = moduleIdentifier;
		this.command = command;
		this.compositeStep = compositeStep;
	}
	
	public CompositeStep getCompositeStep(){
		return this.compositeStep;
	}
	public JsonObject getCommand(){
		return this.command;
	}
	public ModuleIdentifier getModuleIdentifier(){
		return this.moduleIdentifier;
	}
}
