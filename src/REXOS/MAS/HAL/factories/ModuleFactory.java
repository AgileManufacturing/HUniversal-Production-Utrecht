package HAL.factories;

import java.lang.reflect.InvocationTargetException;
import java.sql.SQLException;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.DynamicClassFactory;
import libraries.dynamicloader.InstantiateClassException;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;
import HAL.HardwareAbstractionLayer;
import HAL.JavaSoftware;
import HAL.Module;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.Mutation;
import HAL.RosSoftware;
import HAL.capabilities.Capability;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleExecutingException;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;

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
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM Module \n" + 
			"WHERE equiplet = ? AND \n" + 
			"	attachedToRight = attachedToLeft + 1;"; 
	private static final String getModuleIdentifiersOfPhysicalModuleTreesForCapability = 
			"SELECT * \n" + 
			"FROM CapabilityTypeRequiredMutation AS currentRequiredMutation \n" + 
			"JOIN Module AS currentModule \n" + 
			"WHERE currentRequiredMutation.capabilityType = ? AND \n" + 
			"  currentRequiredMutation.treeNumber = ? AND \n" + 
			"  NOT EXISTS( \n" + 
			"	SELECT * \n" + 
			"	FROM CapabilityTypeRequiredMutation \n" + 
			"	WHERE currentRequiredMutation.capabilityType = capabilityType AND \n" + 
			"	  currentRequiredMutation.treeNumber = treeNumber AND \n" + 
			"	  mutation NOT IN( \n" + 
			"		SELECT mutation \n" + 
			"		FROM SupportedMutation \n" + 
			"		JOIN Module ON SupportedMutation.manufacturer = Module.manufacturer AND \n" +  
			"		  SupportedMutation.typeNumber = Module.typeNumber \n" + 
			"		WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" + 
			"		  currentModule.attachedToRight <= attachedToRight \n AND \n" + 
			"		  currentModule.equiplet = ?" + 
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
			"WHERE ModuleCalibration = ?;";
	private static final String addCalibrationData =
			"INSERT INTO ModuleCalibration \n" + 
			"(date, properties) \n" + 
			"VALUES(?, ?);";
	private static final String addModuleToCalibrationData =
			"INSERT INTO ModuleCalibrationModuleSet \n" + 
			"(ModuleCalibration, manufacturer, typeNumber, serialNumber) \n" + 
			"VALUES(?, ?, ?, ?);";
	private static final String removeAllCalibrationDataForModule =
			"DELETE FROM ModuleCalibration \n" + 
			"WHERE id IN( \n" + 
			"	SELECT ModuleCalibration \n" + 
			"	FROM ModuleCalibrationModuleSet \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"); \n";
	
	
	
	
	private static final String addModuleType =
			"INSERT INTO ModuleType \n" + 
			"(manufacturer, typeNumber, moduleTypeProperties, halSoftware, rosSoftware) \n" +  
			"VALUES (?, ?, ?, ?, ?);"; 
	private static final String getModuleType =
			"SELECT * \n" +
			"FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ?;"; 
	private static final String removeModuleTypesWithNoModules =
			"DELETE FROM ModuleType \n" + 
			"WHERE NOT EXISTS( \n" +  
			"	SELECT * \n" +
			"	FROM Module \n" +
			"	WHERE manufacturer = ModuleType.manufacturer AND \n" + 
			"		typeNumber = ModuleType.typeNumber \n" +
			");";
	
	private static final String addTopModule =
			"INSERT INTO Module \n" + 
			"(manufacturer, typeNumber, serialNumber, moduleProperties, equiplet, mountPointX, mountPointY, attachedToLeft, attachedToRight) \n" +  
			"VALUES (?, ?, ?, ?, ?, ?, ?, (\n" + 
			"	IFNULL( ( \n" + 
			"		SELECT max(attachedToRight) + 1 \n" +
			"		FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	), ( \n" + 
			"		1 \n" + 
			"	) ) \n" + 
			"), ( \n" + 
			"	IFNULL( ( \n" + 
			"		SELECT max(attachedToRight) + 2 \n" +
			"		FROM (SELECT * FROM Module) AS tbl2 \n" + 
			"	), ( \n" + 
			"		2 \n" +  
			"	) ) \n" +  
			"));"; 
	private static final String getModule =
			"SELECT * \n" +
			"FROM Module \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;"; 
	private static final String addModuleAttachedToModule =
			"INSERT INTO Module \n" + 
			"(manufacturer, typeNumber, serialNumber, moduleProperties, equiplet, attachedToLeft, attachedToRight) \n" +  
			"VALUES (?, ?, ?, ?, ?, ( \n" + 
			"	SELECT attachedToLeft + 1 \n" + 
			"	FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"), ( \n" + 
			"	SELECT attachedToLeft + 2 \n" + 
			"	FROM (SELECT * FROM Module) AS tbl2 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"));"; 
	private static final String insertSpaceInNestedTreeForModuleLeft =
			"UPDATE Module \n" + 
			"SET attachedToLeft = attachedToLeft + 2 \n" +  
			"WHERE attachedToLeft >= ( \n" + 
			"	SELECT attachedToRight \n" + 
			"	FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			");"; 
	private static final String insertSpaceInNestedTreeForModuleRight =
			"UPDATE Module \n" + 
			"SET attachedToRight = attachedToRight + 2 \n" +  
			"WHERE attachedToRight >= ( \n" + 
			"	SELECT attachedToRight \n" + 
			"	FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			");"; 
	private static final String removeModule =
			"DELETE FROM Module \n" + 
			"WHERE manufacturer = ? AND \n" +  
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;"; 
	
	private ModuleListener moduleListener;
	private DynamicClassFactory<Module> dynamicClassFactory;
	private HardwareAbstractionLayer hal;
	
	private HashMap<ModuleIdentifier, Module> loadedModules;

	
	public ModuleFactory(ModuleListener moduleListener, HardwareAbstractionLayer hal) throws KnowledgeException{
		super(new KnowledgeDBClient());
		this.moduleListener = moduleListener;
		this.dynamicClassFactory = new DynamicClassFactory<>();
		this.hal = hal;
		this.loadedModules = new HashMap<ModuleIdentifier, Module>();
	}
	public ArrayList<ModuleActor> getBottomModulesForFunctionalModuleTree(Capability capability, int treeNumber) throws FactoryException, JarFileLoaderException{
		ArrayList<ModuleActor> modules = new ArrayList<ModuleActor>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersOfPhysicalModuleTreesForCapability, capability.getName(), treeNumber, hal.getEquipletName());
			for (Row row : rows) {
				String manufacturer = (String) row.get("manufacturer");
				String typeNumber = (String) row.get("typeNumber");
				String serialNumber = (String) row.get("serialNumber");
				ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
				modules.add((ModuleActor) this.getModuleByIdentifier(identifier));
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
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersForBotomModules, hal.getEquipletName());
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
	public void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws FactoryException, JarFileLoaderException, ModuleExecutingException{
		ModuleActor module = (ModuleActor) getModuleByIdentifier(hardwareStep.getModuleIdentifier());
		module.executeHardwareStep(processListener, hardwareStep);
	}
	
	public Module getModuleByIdentifier(ModuleIdentifier moduleIdentifier) throws FactoryException, JarFileLoaderException{
		System.out.println(loadedModules);
		
		for (ModuleIdentifier loadedModuleIdentifier : loadedModules.keySet()) {
			if(moduleIdentifier.equals(loadedModuleIdentifier) == true) {
				return loadedModules.get(loadedModuleIdentifier);
			}
			
		}
		DynamicClassDescription description = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleIdentifier).getDynamicClassDescription();
		try {
			Class<Module> moduleClass = dynamicClassFactory.getClassFromDescription(description);
			Module module = moduleClass.getConstructor(ModuleIdentifier.class, ModuleFactory.class, ModuleListener.class).
					newInstance(moduleIdentifier, this, moduleListener);
			loadedModules.put(moduleIdentifier, module);
			return module;
		} catch (InstantiateClassException | InstantiationException | IllegalAccessException
				| IllegalArgumentException | InvocationTargetException
				| NoSuchMethodException | SecurityException ex) {
			throw new FactoryException("well, we are fucked", ex);
		}
	}
	private boolean isModuleTypeKnown(ModuleIdentifier moduleIdentifier) {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			if(rows.length == 1) {
				return true;
			} else {
				return false;
			}
		} catch (KnowledgeException ex) {
			System.err.println("HAL::ModuleFactory::isModuleTypeKnown(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return false;
		}
	}
	
	public boolean insertModule(JsonObject staticSettings, JsonObject dynamicSettings) {
		try{
			try{
				knowledgeDBClient.getConnection().setAutoCommit(false);
				ModuleIdentifier moduleIdentifier = new ModuleIdentifier(staticSettings.get("manufacturer").getAsString(), 
						staticSettings.get("typeNumber").getAsString(), staticSettings.get("serialNumber").getAsString());
				
				if(isModuleTypeKnown(moduleIdentifier)) {
					updateModuleType(moduleIdentifier, staticSettings.get("type").getAsJsonObject());
				} else {
					insertModuleType(moduleIdentifier, staticSettings.get("type").getAsJsonObject());
				}
				
				String properties = staticSettings.get("properties").getAsString();
				
				if(dynamicSettings.get("attachedTo").isJsonNull()) {
					// we are not attached to another module
					Integer mountPointX = dynamicSettings.get("mountPointX").getAsInt();
					Integer mountPointY = dynamicSettings.get("mountPointY").getAsInt();
					knowledgeDBClient.executeUpdateQuery(addTopModule, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), 
							moduleIdentifier.getSerialNumber(), properties, hal.getEquipletName(), mountPointX, mountPointY);
				}
				else if(dynamicSettings.get("mountPointX").isJsonNull() || dynamicSettings.get("mountPointY").isJsonNull()) {
					// this module is attached to another module
					JsonObject parentModuleJson = dynamicSettings.get("attachedTo").getAsJsonObject();
					ModuleIdentifier parentModuleIdentifier = new ModuleIdentifier(parentModuleJson.get("manufacturer").getAsString(), 
							parentModuleJson.get("typeNumber").getAsString(), parentModuleJson.get("serialNumber").getAsString());
					
					insertSpace(parentModuleIdentifier);
					knowledgeDBClient.executeUpdateQuery(addModuleAttachedToModule, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), 
							moduleIdentifier.getSerialNumber(), properties, hal.getEquipletName(), 
							parentModuleIdentifier.getManufacturer(), parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber(), 
							parentModuleIdentifier.getManufacturer(), parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber());
				}
				else {
					throw new FactoryException("Module both attached to the mountplate and a module");
				}
				
				// calibration
				JsonArray calibrationEntries = staticSettings.get("calibrationData").getAsJsonArray();
				deserializeCalibrationData(calibrationEntries);
				
			} catch(Exception ex) {
				System.err.println("HAL::ModuleFactory::insertModule(): Error occured while inserting module " + ex);
				ex.printStackTrace();
				knowledgeDBClient.getConnection().rollback();
				knowledgeDBClient.getConnection().setAutoCommit(true);
				return false;
			}
			knowledgeDBClient.getConnection().commit();
			knowledgeDBClient.getConnection().setAutoCommit(true);
			return true;
		} catch(SQLException ex) {
			return false;
		}
	}
	public boolean updateModule(JsonObject staticSettings,
			JsonObject dynamicSettings) {
		// TODO Auto-generated method stub
		return false;
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier) throws Exception{
		try{
			JsonObject output = new JsonObject();
			output.addProperty("manufacturer", moduleIdentifier.getManufacturer());
			output.addProperty("typeNumber", moduleIdentifier.getTypeNumber());
			output.addProperty("serialNumber", moduleIdentifier.getSerialNumber());
			
			JsonObject type = new JsonObject();
			Module module = this.getModuleByIdentifier(moduleIdentifier);
			String moduleProperties = module.getProperties();
			type.addProperty("properties", moduleProperties);
			
			// fetch halSoftware
			JavaSoftware halSoftware = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleIdentifier);
			type.add("halSoftware", halSoftware.serialize());
			// fetch rosSoftware
			RosSoftware rosSoftware = RosSoftware.getRosSoftwareForModuleIdentifier(moduleIdentifier);
			System.out.println("RosSoftware ="+rosSoftware.serialize());
			type.add("rosSoftware", rosSoftware.serialize());
			
			type.add("supportedMutations", Mutation.serializeAllSupportedMutations(moduleIdentifier, knowledgeDBClient));
			Mutation.removeSupportedMutations(moduleIdentifier, knowledgeDBClient);
			
			output.add("calibrationData", serializeCalibrationData(moduleIdentifier));
			knowledgeDBClient.executeUpdateQuery(removeAllCalibrationDataForModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			
			Row[] moduleTypeRows = knowledgeDBClient.executeSelectQuery(getModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			type.addProperty("properties", (String) moduleTypeRows[0].get("moduleTypeProperties"));
			
			Row[] moduleRows = knowledgeDBClient.executeSelectQuery(getModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			output.addProperty("properties", (String) moduleRows[0].get("moduleProperties"));
			
			knowledgeDBClient.executeUpdateQuery(removeModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			knowledgeDBClient.executeUpdateQuery(removeModuleTypesWithNoModules);
			
			output.add("type", type);
			return output;
		} catch (KnowledgeException ex) {
			throw new FactoryException("deletion of module failed :(", ex);
		}
	}
	
	private void insertSpace(ModuleIdentifier parentModuleIdentifier) throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleLeft, parentModuleIdentifier.getManufacturer(), 
				parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleRight, parentModuleIdentifier.getManufacturer(), 
				parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber());
		
	}
	private boolean insertModuleType(ModuleIdentifier moduleIdentifier, JsonObject type) throws KnowledgeException {
		JsonObject halSoftwareObject = type.get("halSoftware").getAsJsonObject();
		JavaSoftware halSoftware = JavaSoftware.insertJavaSoftware(halSoftwareObject, knowledgeDBClient);
		int halSoftwareId = halSoftware.getId();
		
		JsonObject rosSoftwareObject = type.get("rosSoftware").getAsJsonObject();
		RosSoftware rosSoftware = RosSoftware.insertRosSoftware(rosSoftwareObject, knowledgeDBClient);
		int rosSoftwareId = rosSoftware.getId();
		
		String properties = type.get("properties").getAsString();
		knowledgeDBClient.executeUpdateQuery(addModuleType, moduleIdentifier.getManufacturer(), 
				moduleIdentifier.getTypeNumber(), properties, halSoftwareId, rosSoftwareId);
		
		JsonArray supportedMutationEntries = type.get("supportedMutations").getAsJsonArray();
		Mutation.insertSupportedMutations(moduleIdentifier, supportedMutationEntries, knowledgeDBClient);
		
		return true;
	}
	private void updateModuleType(ModuleIdentifier moduleIdentifier, JsonObject type) {
		JsonObject halSoftwareObject = type.get("halSoftware").getAsJsonObject();
		JavaSoftware javaSoftware = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		int currentJavaSoftwareBuildNumber = javaSoftware.getBuildNumber();
		
		int newJavaSoftwareBuildNumber = JavaSoftware.getBuildNumber(halSoftwareObject);
		
		if(newJavaSoftwareBuildNumber > currentJavaSoftwareBuildNumber) {
			// update the halSoftware
			javaSoftware.updateJavaSoftware(halSoftwareObject);
		}
		
		JsonObject rosSoftwareObject = type.get("rosSoftware").getAsJsonObject();
		RosSoftware rosSoftware = RosSoftware.getRosSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		int currentRosSoftwareBuildNumber = rosSoftware.getBuildNumber();
		
		int newRosSoftwareBuildNumber = RosSoftware.getBuildNumber(rosSoftwareObject);
		
		if(newRosSoftwareBuildNumber > currentRosSoftwareBuildNumber) {
			// update the halSoftware
			rosSoftware.updateRosSoftware(rosSoftwareObject);
		}
		
	}
	
	private JsonArray serializeCalibrationData(ModuleIdentifier moduleIdentifier) {
		JsonArray calibrationEntries = new JsonArray();
		try {
			Row[] calibrationDataRows = knowledgeDBClient.executeSelectQuery(getAllCalibrationDataForModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			for (Row calibrationDataRow : calibrationDataRows) {
				Integer moduleCalibrationId = (Integer) calibrationDataRow.get("id");
				String dateTime = ((Timestamp) calibrationDataRow.get("date")).toString();
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
					JsonObject moduleSetEntry = moduleSetEntryElement.getAsJsonObject();
					
					String manufacturer = moduleSetEntry.get("manufacturer").getAsString();
					String typeNumber = moduleSetEntry.get("typeNumber").getAsString();
					String serialNumber = moduleSetEntry.get("serialNumber").getAsString();
					knowledgeDBClient.executeUpdateQuery(addModuleToCalibrationData, 
							calibrationDataId, manufacturer, typeNumber, serialNumber);
				}
			}
		} catch (KnowledgeException ex) {
			System.err.println("HAL::ModuleFactory::serializeCalibrationData(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
}