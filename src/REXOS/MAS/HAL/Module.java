package HAL;

import java.util.ArrayList;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import HAL.listeners.ModuleListener;

public abstract class Module { //implements mongolistener
	protected ModuleListener moduleListener;
	protected KnowledgeDBClient knowledgeDBClient;
	protected ModuleIdentifier moduleIdentifier;
	protected String equiplet;
	protected String moduleProperties;
	protected int attachedToLeft;
	protected int attachedToRight;
	protected int mountPointX;
	protected int mountPointY;
	
	public Module(ModuleIdentifier moduleIdentifier) throws KnowledgeException{
		this.moduleIdentifier = moduleIdentifier;
		this.knowledgeDBClient = new KnowledgeDBClient();
	}	
	
	
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	@SuppressWarnings("unused")
	protected Module getParentModule() throws KnowledgeException, KeyNotFoundException{
		//TODO
		
		String sql = "SELECT * FROM Module " +
						"WHERE attachedToLeft < (" +
							"SELECT attachedToLeft FROM Module " +
								"WHERE manufacturer = '" + moduleIdentifier.getManufacturer() +
								"' AND typeNumber = '" + moduleIdentifier.getTypeNumber() +
								"' AND serialNumber = '" + moduleIdentifier.getSerialNumber() +
						"') AND attachedToRight > (" +
 							"SELECT attachedToRight FROM Module " +
 								"WHERE manufacturer = '" + moduleIdentifier.getManufacturer() +
 								"' AND typeNumber = '" + moduleIdentifier.getTypeNumber() +
 								"' AND serialNumber = '" + moduleIdentifier.getSerialNumber() +
 						"') ORDER BY abs(attachedToLeft - attachedToRight) ASC LIMIT 1";
		
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(sql);
		
		ModuleIdentifier moduleIdentifier = new ModuleIdentifier();
		moduleIdentifier.setManufacturer(resultSet[0].get("manufacturer").toString());
		moduleIdentifier.setTypeNumber(resultSet[0].get("typeNumber").toString());
		moduleIdentifier.setSerialNumber(resultSet[0].get("serialNumber").toString());
		ModuleFactory moduleFactory = new ModuleFactory(moduleListener);		
		return moduleFactory.getModuleByIdentifier(moduleIdentifier);
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
