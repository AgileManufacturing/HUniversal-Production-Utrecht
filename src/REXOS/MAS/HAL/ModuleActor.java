package HAL;

import java.util.ArrayList;

import HAL.listeners.ModuleListener;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

public abstract class ModuleActor extends Module {//implements mongolistener
	protected ModuleListener moduleListener;
	
	public ModuleActor(ModuleIdentifier moduleIdentifier) throws KnowledgeException {
		super(moduleIdentifier);
	}
	
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	
	abstract public void executeHardwareStep(HardwareStep hardwareStep);
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws KnowledgeException, KeyNotFoundException;
	
	public void onHardwareStepChanged(String state, long hardwareStepSerialId){
		moduleListener.onProcessStateChanged(state, hardwareStepSerialId, this);
	}
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
}
