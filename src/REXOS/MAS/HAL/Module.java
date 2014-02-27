package HAL;

import java.sql.ResultSet;
import java.util.ArrayList;

import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import HAL.listeners.ModuleListener;

public abstract class Module { //implements mongolistener
	protected ModuleListener moduleListener;
	protected KnowledgeDBClient knowledgeDBClient;
	protected String manufacturer;
	protected String typeNumber;
	protected String serialNumber;
	protected String equiplet;
	protected String moduleProperties;
	protected int attachedToLeft;
	protected int attachedToRight;
	protected int mountPointX;
	protected int mountPointY;
	
	public Module(){
		setRequiredAttributes();
		try {
			this.knowledgeDBClient = new KnowledgeDBClient();
		} catch (KnowledgeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	abstract protected void setRequiredAttributes();	
	
	
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	@SuppressWarnings("unused")
	private Module getParentModule(){
		//TODO
		
		String sql = "SELECT * FROM Module " +
						"WHERE attachedToLeft < (" +
							"SELECT attachedToLeft FROM Module " +
								"WHERE manufacturer = '" + manufacturer +
								"' AND typeNumber = '" + typeNumber +
								"' AND serialNumber = '" + serialNumber +
						"') AND attachedToRight > (" +
 							"SELECT attachedToRight FROM Module " +
 								"WHERE manufacturer = '" + manufacturer +
 								"' AND typeNumber = '" + typeNumber +
 								"' AND serialNumber = '" + serialNumber +
 						"') ORDER BY abs(attachedToLeft - attachedToRight) ASC LIMIT 1";
		try {
			Row[] resultSet = knowledgeDBClient.executeSelectQuery(sql);
			
		} catch (KnowledgeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	abstract public void executeHardwareStep(HardwareStep hardwareStep);
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep);
	
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
