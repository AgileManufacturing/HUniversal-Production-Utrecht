package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import HAL.exceptions.HardwareAbstractionLayerProcessException;
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.listeners.ModuleListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

public abstract class ModuleActor extends Module {//implements mongolistener
	protected ModuleListener moduleListener;
	protected BlackboardClient mongoClient;
	protected static final String MONGO_HOST = "145.89.191.131";
	
	public ModuleActor(ModuleIdentifier moduleIdentifier) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier);
		mongoClient = new BlackboardClient(MONGO_HOST);
	}
	
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	
	abstract public void executeHardwareStep(HardwareStep hardwareStep) throws ModuleExecutingException;
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws KnowledgeException, KeyNotFoundException, ModuleTranslatingException;
	
	public void onHardwareStepChanged(String state, long hardwareStepSerialId) throws HardwareAbstractionLayerProcessException{
		moduleListener.onProcessStateChanged(state, hardwareStepSerialId, this);
	}
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
}
