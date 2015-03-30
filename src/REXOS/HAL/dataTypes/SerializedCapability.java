package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class SerializedCapability implements Serializable{
	private static final long serialVersionUID = -6760054619972669658L;
	
	public static final String NAME = "name";
	public static final String HAL_SOFTWARE = "halSoftware";
	public static final String REQUIRED_MUTATION_TREES = "requiredMutationsTrees";
	public static final String SERVICES = "services";
	
	/**
	 * SQL query for selecting all the data of moduleType.
	 * Input: moduleTypeManufacturer, moduleTypeTypeNumber
	 */
	private static final String getCapabilityType = 
			"SELECT * \n" + 
			"FROM CapabilityType \n" + 
			"WHERE name = ?;";
	/**
	 * SQL query for selecting the serviceTypes for a capabilityType.
	 * Input: capabilityTypeName
	 */
	private static final String getServiceTypesForCapabilityType = 
			"SELECT serviceType \n" + 
			"FROM ServiceType_CapabilityType \n" + 
			"WHERE capabilityType = ?;";
	/**
	 * SQL query for selecting all the associated capabilityTypes to a ModuleIdentifier.
	 * Input: ModuleIdentifierManufacturer, ModuleIdentifierTypeNumber
	 * A capabilityTypes is considered associated when at least one required mutation matches with a supported mutation of this module type (which is identified with by
	 * {@link ModuleIdentifier}).
	 */
	private static final String getAllAssociatedCapabilityTypesForModuleIdentifier = 
			"SELECT DISTINCT capabilityType \n" + 
			"FROM CapabilityTypeRequiredMutation \n" +
			"WHERE mutation IN( \n" + 
			"	SELECT mutation \n" + 
			"	FROM SupportedMutation \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");";
	/**
	 * SQL query for adding a capabilityType.
	 * Input: capabilityTypeName, halSoftwareId
	 */
	private static final String addCapabilityType = 
			"INSERT IGNORE INTO CapabilityType \n" + 
			"(name, halSoftware) \n" + 
			"VALUES(?, ?);";
	/**
	 * SQL query for adding a serviceType.
	 * Input: serviceTypeName
	 */
	private static final String addServiceType = 
			"INSERT IGNORE INTO ServiceType \n" + 
			"(name) \n" + 
			"VALUES(?);";
	/**
	 * SQL query for adding a relation between a serviceType and a capabilityType.
	 * Input: serviceTypeName, capabilityTypeName
	 */
	private static final String addServiceType_CapabilityType = 
			"INSERT IGNORE INTO ServiceType_CapabilityType \n" + 
			"(serviceType,capabilityType) \n" + 
			"VALUES(?, ?);";
	private static final String removeCapability = 
			"DELETE FROM CapabilityType \n" +
			"WHERE name = ?;";
	/**
	 * SQL query for removing all the capabilityType which are obsolete.
	 * Input: -
	 * A capabilityType is considered to be obsolete if there are no modulesTypes supporting a mutation that this capabilityType requires.
	 */
	private static final String getCapabilitiesWithNoModuleTypes = 
			"SELECT name \n" +
			"FROM CapabilityType \n" +
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM CapabilityTypeRequiredMutation \n" +
			"	WHERE mutation IN ( \n" +
			"		SELECT mutation \n" +
			"		FROM SupportedMutation \n" +
			"	) AND \n" +
			"	CapabilityTypeRequiredMutation.capabilityType = CapabilityType.name" +
			");";
	
	
	public String name;
	public JavaSoftware halSoftware;
	public ArrayList<MutationTree> requiredMutationTrees;
	public ArrayList<String> supportedServices;
	
	public SerializedCapability() {
		requiredMutationTrees = new ArrayList<MutationTree>();
		supportedServices = new ArrayList<String>();
	}
	
	public static SerializedCapability getCapability(String capabilityName, KnowledgeDBClient knowledgeDBClient) {
		SerializedCapability output = new SerializedCapability();
		
		output.name = capabilityName;
		output.halSoftware = JavaSoftware.getJavaSoftwareForCapabilityName(capabilityName);
		output.requiredMutationTrees = MutationTree.getMutationTreesForCapabilityName(capabilityName, knowledgeDBClient);
		output.supportedServices = SerializedCapability.getServiceTypes(capabilityName, knowledgeDBClient);
		
		return output;
	}
	public static ArrayList<SerializedCapability> getCapabilitiesFromDatabase(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		ArrayList<String> capabilityNames = new ArrayList<String>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getAllAssociatedCapabilityTypesForModuleIdentifier, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		for (Row row : rows) {
			capabilityNames.add((String) row.get("capabilityType"));
		}
		
		ArrayList<SerializedCapability> output = new ArrayList<SerializedCapability>();
		for (String capabilityName : capabilityNames) {
			output.add(getCapability(capabilityName, knowledgeDBClient));
		}
		return output;
	}
	
	public static SerializedCapability deSerialize(JSONObject input) throws JSONException {
		SerializedCapability output = new SerializedCapability();
		
		output.name = input.getString(NAME);
		output.halSoftware = JavaSoftware.deSerialize(input.getJSONObject(HAL_SOFTWARE));
		
		JSONArray mutationTrees = input.getJSONArray(REQUIRED_MUTATION_TREES);
		for (int i = 0; i < mutationTrees.length(); i++) {
			output.requiredMutationTrees.add(MutationTree.deSerialize(mutationTrees.getJSONObject(i), output.name));
		}
		
		JSONArray services = input.getJSONArray(SERVICES);
		for (int i = 0; i < services.length(); i++) {
			output.supportedServices.add(services.getString(i));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(NAME, name);
		output.put(HAL_SOFTWARE, halSoftware.serialize());
		
		JSONArray mutationTrees = new JSONArray();
		for (int i = 0; i < this.requiredMutationTrees.size(); i++) {
			mutationTrees.put(this.requiredMutationTrees.get(i).serialize());
		}
		output.put(REQUIRED_MUTATION_TREES, mutationTrees);
		
		JSONArray services = new JSONArray();
		for (int i = 0; i < this.supportedServices.size(); i++) {
			services.put(supportedServices.get(i));
		}
		output.put(SERVICES, services);
		
		return output;
	}
	
	/**
	 * This method will insert a array of capabilityTypes into the knowledge database, using the data provided in the JSONArray.
	 * 
	 * @param capabilityTypes
	 * @return true if successful, false otherwise
	 * @throws JSONException
	 */
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		if(existsInDatabase(knowledgeDBClient) == true) {
			updateDatabase(knowledgeDBClient);
		} else {
			int halSoftwareId = halSoftware.insertIntoDatabase(knowledgeDBClient);
			
			
			knowledgeDBClient.executeUpdateQuery(addCapabilityType, name, halSoftwareId);
			
			for (MutationTree requiredMutationTree : requiredMutationTrees) {
				requiredMutationTree.insertIntoDatabase(name, knowledgeDBClient);
			}
	
			for (String service : supportedServices) {
				knowledgeDBClient.executeUpdateQuery(addServiceType, service);
				knowledgeDBClient.executeUpdateQuery(addServiceType_CapabilityType, service, name);
				
			}
		}
	}
	private void updateDatabase(KnowledgeDBClient knowledgeDBClient) {
		JavaSoftware currentJavaSoftware = JavaSoftware.getJavaSoftwareForCapabilityName(name);
		int newJavaBuildNumber = halSoftware.buildNumber;
		int currentJavaBuildNumber = currentJavaSoftware.buildNumber;
		if (newJavaBuildNumber > currentJavaBuildNumber) {
			// update the halSoftware
			Logger.log(LogSection.HAL_RECONFIG, LogLevel.INFORMATION, "Updating HAL software for capability " + name);
			halSoftware.updateDatabase(currentJavaSoftware, knowledgeDBClient);
		}
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		for (MutationTree tree : requiredMutationTrees) {
			tree.removeFromDatabase(knowledgeDBClient);
		}
		
		knowledgeDBClient.executeUpdateQuery(removeCapability, name);
		halSoftware.removeFromDatabase(knowledgeDBClient);
	}
	/**
	 * This method will serialize all the capabilityTypes associated with the moduleType (which is identified by the {@link ModuleIdentifier}).
	 * This method will also remove all the capabilityTypes which have become obsolete after removing the module type.
	 * A capabilityType is considered to be obsolete if none of the required mutations matches a supported mutation.
	 * 
	 * @param moduleIdentifier
	 * @return The serialized associated capabilities.
	 */
	public static void removeAllUnusedCapabilitiesFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		ArrayList<String> capabilityNames = new ArrayList<String>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getCapabilitiesWithNoModuleTypes);
		for (Row row : rows) {
			capabilityNames.add((String) row.get("name"));
		}
		
		for (String capabilityName : capabilityNames) {
			SerializedCapability capability = getCapability(capabilityName, knowledgeDBClient);
			capability.removeFromDatabase(knowledgeDBClient);
		}
	}
	/**
	 * This method determines if a moduleType (identified by the {@link ModuleIdentifier}) is known in the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @return true if the moduleType is known in the knowledge database, false otherwise.
	 */
	public boolean existsInDatabase(KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getCapabilityType, name);
		if (rows.length == 1) {
			return true;
		} else {
			return false;
		}
	}
	
	public static ArrayList<String> getServiceTypes(String capabilityName, KnowledgeDBClient knowledgeDBClient) {
		ArrayList<String> output = new ArrayList<String>(); 
		
		Row[] serviceRows = knowledgeDBClient.executeSelectQuery(getServiceTypesForCapabilityType, capabilityName);
		for (Row serviceRow : serviceRows) {
			output.add((String) serviceRow.get("serviceType"));
		}
		
		return output;
		
	}
}
