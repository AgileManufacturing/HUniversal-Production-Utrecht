package HAL;

import java.util.ArrayList;

import HAL.exceptions.FactoryException;
import HAL.factories.ModuleFactory;
import HAL.listeners.BlackboardListener;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

public abstract class Module implements BlackboardListener{ 
	protected KnowledgeDBClient knowledgeDBClient;
	protected ModuleIdentifier moduleIdentifier;
	protected ModuleFactory moduleFactory;
	protected ModuleListener moduleListener;
	protected ProcessListener processListener;
	
	public Module(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) 
			throws KnowledgeException{
		this.moduleIdentifier = moduleIdentifier;
		this.knowledgeDBClient = new KnowledgeDBClient();
		this.moduleFactory = moduleFactory;
		this.moduleListener = moduleListener;
		
		moduleFactory.getHAL().getBlackBoardHandler().addBlackboardListener(this);
	}	
	
	public ModuleIdentifier getModuleIdentifier(){
		return this.moduleIdentifier;
	}
	
	public ArrayList<Integer> getMountPosition() {
		try{
			String sql = "SELECT mountPointX, mountPointY FROM Module " +
					"WHERE manufacturer = '" + moduleIdentifier.getManufacturer() +
					"' AND typeNumber = '" + moduleIdentifier.getTypeNumber() +
					"' AND serialNumber = '" + moduleIdentifier.getSerialNumber() +
					"'";
			Row[] resultSet = knowledgeDBClient.executeSelectQuery(sql);
			if (resultSet.length == 1){
				ArrayList<Integer> position = new ArrayList<Integer>();
				position.add((Integer) resultSet[0].get("mountPointX"));
				position.add((Integer) resultSet[0].get("mountPointY"));
				return position;
			}
			return null;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	public Module getParentModule() throws FactoryException, JarFileLoaderException {
		try{
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
			
			if (resultSet.length >= 1){
				ModuleIdentifier moduleIdentifier = new ModuleIdentifier(
						resultSet[0].get("manufacturer").toString(),
						resultSet[0].get("typeNumber").toString(),
						resultSet[0].get("serialNumber").toString());
				return this.moduleFactory.getModuleByIdentifier(moduleIdentifier);
			}
			else return null;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			// this is impossible!
			System.err.println("HAL::Module::getParentModule(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}


	public String getProperties() {
		// TODO Auto-generated method stub
		return null;
	}
	
	
	@Override
	public void OnEquipletStateChanged(String equipletName, String state) {
		// ignore
	}

	@Override
	public void OnEquipletModeChanged(String equipletName, String mode) {
		// ignore
	}

	@Override
	public void OnEquipletIpChanged(String ip) {
		// ignore
	}
}
