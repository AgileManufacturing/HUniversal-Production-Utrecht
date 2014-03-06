package HAL;

import java.io.Serializable;

import com.google.gson.Gson;

public class HardwareStep implements Serializable {
	private static final long serialVersionUID = 3031019032556167945L;
	private CompositeStep compositeStep;
	private Gson command;
	private ModuleIdentifier moduleIdentifier;
	
	public HardwareStep(CompositeStep compositeStep, Gson command, ModuleIdentifier moduleIdentifier){
		this.moduleIdentifier = moduleIdentifier;
		this.command = command;
		this.compositeStep = compositeStep;
	}
	
	public CompositeStep getCompositeStep(){
		return this.compositeStep;
	}
	public Gson getCommand(){
		return this.command;
	}
	public ModuleIdentifier getModuleIdentifier(){
		return this.moduleIdentifier;
	}
}
