package HAL;

import java.util.ArrayList;

import HAL.factories.ModuleFactory;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

public abstract class Module { 
	protected KnowledgeDBClient knowledgeDBClient;
	protected ModuleIdentifier moduleIdentifier;
	protected ModuleFactory moduleFactory;
	
	public Module(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory) throws KnowledgeException{
		this.moduleIdentifier = moduleIdentifier;
		this.knowledgeDBClient = new KnowledgeDBClient();
		this.moduleFactory = moduleFactory;
	}	
	
	public ModuleIdentifier getModuleIdentifier(){
		return this.moduleIdentifier;
	}
	
	public ArrayList<Integer> getMountPosition() throws KnowledgeException, KeyNotFoundException{
		String sql = "SELECT * FROM Module " +
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
	}
	
	public Module getParentModule() throws Exception {
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
		
		if (resultSet.length == 1){
			ModuleIdentifier moduleIdentifier = new ModuleIdentifier(resultSet[0].get("manufacturer").toString(),
																	 resultSet[0].get("typeNumber").toString(),
																	 resultSet[0].get("serialNumber").toString()
																	);	
			return this.moduleFactory.getModuleByIdentifier(moduleIdentifier);
		}
		else return null;
	}
}
