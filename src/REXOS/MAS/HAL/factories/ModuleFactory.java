package HAL.factories;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

import org.apache.commons.codec.binary.Base64;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.DynamicClassFactory;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;
import HAL.Capability;
import HAL.HardwareAbstractionLayer;
import HAL.HardwareStep;
import HAL.Module;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.listeners.ModuleListener;

public class ModuleFactory extends Factory {
	// SQL queries
	private static final String getModuleIdentifiersForCapability = 
			"SELECT * \n" + 
			"FROM CapabilityTypeDependencySet AS currentDependencySet \n" + 
			"JOIN Module AS currentModule \n" + 
			"WHERE currentDependencySet.capabilityType = ? AND \n" + 
			"  currentDependencySet.Equiplet = ? \n" + 
			"  NOT EXISTS( \n" + 
			"	SELECT * \n" + 
			"	FROM CapabilityTypeDependencySet \n" + 
			"	WHERE currentDependencySet.capabilityType = capabilityType AND \n" + 
			"	  currentDependencySet.treeNumber = treeNumber AND \n" + 
			"	  commandType NOT IN( \n" + 
			"		SELECT commandType \n" + 
			"		FROM ModuleCommandType \n" + 
			"		JOIN Module ON ModuleCommandType.manufacturer = Module.manufacturer AND \n" +  
			"		  ModuleCommandType.typeNumber = Module.typeNumber \n" + 
			"		WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" + 
			"		  currentModule.attachedToRight <= attachedToRight \n" + 
			"	  ) \n" + 
			"  ) AND \n" + 
			"  currentModule.attachedToRight = currentModule.attachedToLeft + 1; \n";

	private static final String getModuleIdentifiersForBotomModules = 
			"SELECT attachedToLeft, attachedToRight, Equiplet \n" + 
			"FROM Module" + 
			"WHERE currentModule.Equiplet = ? AND" + 
			"	attachedToRight = attachedToLeft + 1;"; 
	private static final String getModuleIdentifiersOfPhysicalModuleTreesForCapability = 
			"SELECT * \n" + 
			"FROM CapabilityTypeDependencySet AS currentDependencySet \n" + 
			"JOIN Module AS currentModule \n" + 
			"WHERE currentDependencySet.capabilityType = ? AND \n" + 
			"  currentDependencySet.treeNumber = ? AND \n" + 
			"  currentDependencySet.Equiplet = ? \n" + 
			"  NOT EXISTS( \n" + 
			"	SELECT * \n" + 
			"	FROM CapabilityTypeDependencySet \n" + 
			"	WHERE currentDependencySet.capabilityType = capabilityType AND \n" + 
			"	  currentDependencySet.treeNumber = treeNumber AND \n" + 
			"	  commandType NOT IN( \n" + 
			"		SELECT commandType \n" + 
			"		FROM ModuleCommandType \n" + 
			"		JOIN Module ON ModuleCommandType.manufacturer = Module.manufacturer AND \n" +  
			"		  ModuleCommandType.typeNumber = Module.typeNumber \n" + 
			"		WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" + 
			"		  currentModule.attachedToRight <= attachedToRight \n" + 
			"	  ) \n" + 
			"  ) AND \n" + 
			"  currentModule.attachedToRight = currentModule.attachedToLeft + 1; \n";
	private static final String getAllCalibrationDataForModule =
			"SELECT id, date, properties \n" + 
			"FROM ModuleCalibration \n" + 
			"WHERE id IN ( \n" +
			"	SELECT ModuleCalibration \n" + 
			"	FROM ModuleCalibrationModuleSet \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"); \n";
	private static final String getModuleSetForCalibrationData =
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM ModuleCalibrationModuleSet \n" + 
			"WHERE id = ?;";
	private static final String addCalibrationData =
			"INSERT INTO ModuleCalibration \n" + 
			"(date, properties) \n" + 
			"VALUES(?, ?);";
	private static final String addModuleToCalibrationData =
			"INSERT INTO ModuleCalibrationModuleSet \n" + 
			"(ModuleCalibration, manufacturer, typeNumber, serialNumber) \n" + 
			"VALUES(?, ?, ?, ?);";
	private static final String removeAllCalibrationDataForModuleType =
			"DELETE FROM ModuleCalibration \n" + 
			"WHERE id = ?;";
	
	private static final String getSupportedMutationsForModuleType =
			"SELECT mutation \n" + 
			"FROM SupportedMutation \n" + 
			"WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ?;";
	private static final String addSupportedMutationForModuleType =
			"INSERT INTO SupportedMutation \n" + 
			"(manufacturer, typeNumber, mutation) \n" + 
			"VALUES(?, ?, ?);";
	private static final String removeAllSupportedMutationsForModuleType =
			"DELETE FROM SupportedMutation \n" + 
			"WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ?;";
	private static final String getHalSoftwareForModuleType =
			"SELECT * \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getRosSoftwareForModuleType =
			"SELECT * \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	
	private DynamicClassFactory<Module> dynamicClassFactory;
	private HardwareAbstractionLayer hal;
	
	public ModuleFactory(ModuleListener moduleListener, HardwareAbstractionLayer hal) throws KnowledgeException{
		super(new KnowledgeDBClient());
		this.dynamicClassFactory = new DynamicClassFactory<>(this);
		this.hal = hal;
	}
	public ArrayList<Module> getBottomModulesForFunctionalModuleTree(Capability capability, int treeNumber) throws Exception{
		ArrayList<Module> modules = new ArrayList<Module>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersOfPhysicalModuleTreesForCapability, capability.getName(), treeNumber, hal);
			for (Row row : rows) {
				String manufacturer = (String) row.get("manufacturer");
				String typeNumber = (String) row.get("typeNumber");
				String serialNumber = (String) row.get("serialNumber");
				ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
				modules.add(this.getModuleByIdentifier(identifier));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::ModuleFactory::getBottomModules(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
		
		return modules;
	}
	public ArrayList<Module> getBottomModules() throws Exception {
		ArrayList<Module> modules = new ArrayList<Module>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersForBotomModules);
			for (Row row : rows) {
				String manufacturer = (String) row.get("manufacturer");
				String typeNumber = (String) row.get("typeNumber");
				String serialNumber = (String) row.get("serialNumber");
				
				ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
				modules.add(this.getModuleByIdentifier(identifier));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			// this is impossible!
			System.err.println("HAL::ModuleFactory::getBottomModules(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
		
		return modules;
		
	}
	public void executeHardwareStep(HardwareStep hardwareStep){
		
	}
	public Module getModuleByIdentifier(ModuleIdentifier moduleIdentifier) throws Exception{
		DynamicClassDescription description = new DynamicClassDescription(1, "a");
		Class<Module> moduleClass = dynamicClassFactory.getClassFromDescription(description);
		try {
			return moduleClass.getConstructor(ModuleIdentifier.class).newInstance(moduleIdentifier);
		} catch (InstantiationException | IllegalAccessException
				| IllegalArgumentException | InvocationTargetException
				| NoSuchMethodException | SecurityException ex) {
			throw new Exception("well, we are fucked", ex);
		}
	}
	
	public ArrayList<ModuleActor> getBottomModuleActors(){
		return null;
		
	}
	public boolean insertModule(ModuleIdentifier moduleIdentifier, byte[] rosSoftware, 
			byte[] halSoftware, JsonObject dynamicSettings, JsonObject staticSettings) {
		return true;
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier, byte[] rosSoftware, byte[] halSoftware, JsonObject dynamicSettings){
		return true;
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier){
		try{
			JsonObject output = new JsonObject();
			output.addProperty("manufacturer", moduleIdentifier.getManufacturer());
			output.addProperty("typeNumber", moduleIdentifier.getTypeNumber());
			output.addProperty("serialNumber", moduleIdentifier.getSerialNumber());
			
			JsonObject type = new JsonObject();
			Module module = this.getModuleByIdentifier(moduleIdentifier);
			String moduleProperties = module.getProperties();
			type.addProperty("properties", moduleProperties);
			
			JsonObject halSoftware = new JsonObject();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getHalSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			halSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
			halSoftware.addProperty("className", (String) rows[0].get("className"));
			byte[] jarFile = (byte[]) rows[0].get("jarFile");
			halSoftware.addProperty("jarFile", new String(Base64.encodeBase64(jarFile)));
			type.add("halSoftware", halSoftware);
			
			JsonObject rosSoftware = new JsonObject();
			Row[] rows2 = knowledgeDBClient.executeSelectQuery(getHalSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			rosSoftware.addProperty("buildNumber", (Integer) rows2[0].get("buildNumber"));
			rosSoftware.addProperty("className", (String) rows2[0].get("className"));
			byte[] rosFile = (byte[]) rows2[0].get("rosFile");
			rosSoftware.addProperty("rosFile", new String(Base64.encodeBase64(rosFile)));
			type.add("rosSoftware", rosSoftware);
			
			type.add("supportedMutations", serializeCalibrationData(moduleIdentifier));
			knowledgeDBClient.executeUpdateQuery(removeAllSupportedMutationsForModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			
			type.add("calibrationData", serializeCalibrationData(moduleIdentifier));
			
			
		
	}
	
	private JsonArray serializeCalibrationData(ModuleIdentifier moduleIdentifier) {
		JsonArray calibrationEntries = new JsonArray();
		try {
			Row[] calibrationDataRows = knowledgeDBClient.executeSelectQuery(getAllCalibrationDataForModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			for (Row calibrationDataRow : calibrationDataRows) {
				Integer moduleCalibrationId = (Integer) calibrationDataRow.get("id");
				String dateTime = (String) calibrationDataRow.get("date");
				String properties = (String) calibrationDataRow.get("properties");
				
				JsonObject calibrationDataEntry = new JsonObject();
				calibrationDataEntry.addProperty("date", dateTime);
				calibrationDataEntry.addProperty("data", properties);
				
				// fetch the moduleSet for the calibration data
				// TODO prepare the statement!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				JsonArray moduleEntries = new JsonArray();
				Row[] moduleSetrows = knowledgeDBClient.executeSelectQuery(getModuleSetForCalibrationData, moduleCalibrationId);
				for (Row moduleSetrow : moduleSetrows) {
					String manufacturer = (String) moduleSetrow.get("manufacturer");
					String typeNumber = (String) moduleSetrow.get("typeNumber");
					String serialNumber = (String) moduleSetrow.get("serialNumber");
					
					JsonObject moduleEntry = new JsonObject();
					moduleEntry.addProperty("manufacturer", manufacturer);
					moduleEntry.addProperty("typeNumber", typeNumber);
					moduleEntry.addProperty("serialNumber", serialNumber);
					
					moduleEntries.add(moduleEntry);
				}
				calibrationDataEntry.add("moduleSet", moduleEntries);
				
				calibrationEntries.add(calibrationDataEntry);
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::ModuleFactory::serializeCalibrationData(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
		return calibrationEntries;
	}
	private void deserializeCalibrationData(JsonArray moduleCalibrationEntries) {
		try {
			for (JsonElement moduleCalibrationEntryElement : moduleCalibrationEntries) {
				JsonObject moduleCalibrationEntry = moduleCalibrationEntryElement.getAsJsonObject();
				String dateTime = moduleCalibrationEntry.get("date").getAsString();
				String properties = moduleCalibrationEntry.get("data").getAsString();
				int calibrationDataId = knowledgeDBClient.executeUpdateQuery(addCalibrationData, dateTime, properties);
				
				JsonArray moduleSetEntries = moduleCalibrationEntry.get("moduleSet").getAsJsonArray();
				for (JsonElement moduleSetEntryElement : moduleSetEntries) {
					JsonObject moduleSetEntry = moduleCalibrationEntryElement.getAsJsonObject();
					
					String manufacturer = moduleSetEntry.get("manufacturer").getAsString();
					String typeNumber = moduleSetEntry.get("typeNumber").getAsString();
					String serialNumber = moduleSetEntry.get("serialNumber").getAsString();
					knowledgeDBClient.executeUpdateQuery(addModuleToCalibrationData, 
							calibrationDataId, manufacturer, typeNumber, serialNumber);
				}
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::ModuleFactory::serializeCalibrationData(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
	
	private JsonArray serializeSupportedMutations(ModuleIdentifier moduleIdentifier) {
		JsonArray supportedMutationEntries = new JsonArray();
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedMutationsForModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			for (Row row : rows) {
				String mutation = (String) row.get("mutation");
				
				JsonObject supportedMutationEntry = new JsonObject();
				supportedMutationEntries.add(new JsonPrimitive(mutation));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::ModuleFactory::serializeCalibrationData(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
		return supportedMutationEntries;
}
	private void deserializeSupportedMutations(ModuleIdentifier moduleIdentifier, JsonArray supportedMutationEntries) {
		try {
			for (JsonElement supportedMutationEntryElement : supportedMutationEntries) {
				String mutation = supportedMutationEntryElement.getAsString();
				knowledgeDBClient.executeUpdateQuery(addCalibrationData, 
						moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), mutation);
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::ModuleFactory::serializeCalibrationData(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
}