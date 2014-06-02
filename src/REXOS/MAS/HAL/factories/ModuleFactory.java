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

/**
 * The ModuleFactory is the factory for the {@link Module}s. 
 * It does not only instantiate capabilities using the {@link DynamicClassFactory} (allowing dynamic addition of classes) but also manages part of the knowledge database.
 * @author Tommas Bakker
 *
 */
public class ModuleFactory extends Factory {
	// SQL queries
	/**
	 * SQL query for selecting the moduleIdentifiers of the modules which are bottomModules for an equiplet
	 * Input: equipletName
	 * A module is considered to be a bottomModule when is has no children.
	 */
	private static final String getModuleIdentifiersForBotomModules = 
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM Module \n" + 
			"WHERE equiplet = ? AND \n" + 
			"	attachedToRight = attachedToLeft + 1;"; 
	/**
	 * SQL query for selecting the moduleIdentifiers of the physicalModuleTrees for a functionalModuleTree of a capabilityType on an equiplet
	 * Input: capabilityTypeName, capabilityTypeFunctionalTreeNumber, equipletName
	 * A physicalModuleTree is identified by the bottomModule of that tree.
	 * A physicalModuleTree is suitable for a functionalModuleTree if all the required mutations could be matched with a supported mutation. 
	 */
	private static final String getModuleIdentifiersOfPhysicalModuleTreesForFunctionalModuleTreeOfACapabilityType = 
			"SELECT DISTINCT currentModule.manufacturer, currentModule.typeNumber, currentModule.serialNumber \n" + 
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
	/**
	 * SQL query for selecting all the associated ModuleCalibrationData for a module (which is identified using a {@link ModuleIdentifier}).
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 * ModuleCalibrationData is associated when at least one of the ModuleIdentifiers in the module set matches the ModuleIdentifier of this module.
	 */
	private static final String getAllModuleCalibrationDataForModule =
			"SELECT id, date, properties \n" + 
			"FROM ModuleCalibration \n" + 
			"WHERE id IN ( \n" +
			"	SELECT ModuleCalibration \n" + 
			"	FROM ModuleCalibrationModuleSet \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"); \n";
	/**
	 * SQL query for selecting all the {@link ModuleIdentifier} in the moduleSet of the ModuleCalibrationData.
	 * Input: ModuleCalibrationId
	 */
	private static final String getModuleSetForModuleCalibrationData =
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM ModuleCalibrationModuleSet \n" + 
			"WHERE ModuleCalibration = ?;";
	/**
	 * SQL query for adding ModuleCalibrationData.
	 * Input: date, properties
	 */
	private static final String addModuleCalibrationData =
			"INSERT INTO ModuleCalibration \n" + 
			"(date, properties) \n" + 
			"VALUES(?, ?);";
	/**
	 * SQL query for adding ModuleCalibrationData.
	 * Input: ModuleCalibrationDataId, moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 */
	private static final String addModuleToCalibrationData =
			"INSERT INTO ModuleCalibrationModuleSet \n" + 
			"(ModuleCalibration, manufacturer, typeNumber, serialNumber) \n" + 
			"VALUES(?, ?, ?, ?);";
	/**
	 * SQL query for removing ModuleCalibrationData associated with a module (which is identified using a {@link ModuleIdentifier}).
	 * This effectively removes obsolete ModuleCalibrationData.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 * ModuleCalibrationData is associated when at least one of the ModuleIdentifiers in the module set matches the ModuleIdentifier of this module.
	 * ModuleCalibrationData is considered to be obsolete when at least one of the ModuleIdentifiers in the no longer matches the ModuleIdentifier of the modules attached to the equiplet.
	 */
	private static final String removeAllCalibrationDataForModule =
			"DELETE FROM ModuleCalibration \n" + 
			"WHERE id IN( \n" + 
			"	SELECT ModuleCalibration \n" + 
			"	FROM ModuleCalibrationModuleSet \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			"); \n";
	
	/**
	 * SQL query for adding a moduleType.
	 * Input: moduleTypeManufacturer, moduleTypeTypeNumber, halSoftwareId, rosSoftwareId
	 * Ignored if a record with the same primairy key already exists.
	 */
	private static final String addModuleType =
			"INSERT IGNORE INTO ModuleType \n" + 
			"(manufacturer, typeNumber, moduleTypeProperties, halSoftware, rosSoftware) \n" +  
			"VALUES (?, ?, ?, ?, ?);"; 
	/**
	 * SQL query for selecting all the data of moduleType.
	 * Input: moduleTypeManufacturer, moduleTypeTypeNumber
	 */
	private static final String getModuleType =
			"SELECT * \n" +
			"FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ?;"; 
	/**
	 * SQL query for removing all the moduleType which are obsolete.
	 * Input: -
	 * A moduleType is considered to be obsolete if there are no modules of that type connected to any equiplet.
	 */
	private static final String removeModuleTypesWithNoModules =
			"DELETE FROM ModuleType \n" + 
			"WHERE NOT EXISTS( \n" +  
			"	SELECT * \n" +
			"	FROM Module \n" +
			"	WHERE manufacturer = ModuleType.manufacturer AND \n" + 
			"		typeNumber = ModuleType.typeNumber \n" +
			");";
	
	/**
	 * SQL query for adding a module which is connected to the mountPlate. 
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber, moduleProperties, equiplet, mountPointX, mountPointY
	 * The module is added to the right of the nested set tree.
	 */
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
	/**
	 * SQL query for selecting all the data of a module 
	 * Input: moduleManufacturer, moduleTypeNumber
	 */
	private static final String getModule =
			"SELECT * \n" +
			"FROM Module \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;"; 
	/**
	 * SQL query for adding a module which is connected to another module. The space required is the nested set tree is NOT inserted.
	 * Input: parentModuleManufacturer, parentModuleTypeNumber, parentModuleSerialNumber, parentModuleManufacturer, parentModuleTypeNumber, parentModuleSerialNumber
	 * The module is added to the left of all the children of the parent module.
	 */
	// TODO store the input params so they dont have to be specified twice
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
	/**
	 * SQL query for inserting the left space in the nested set tree for a module which is connected to another module. 
	 * Input: parentModuleManufacturer, parentModuleTypeNumber, parentModuleSerialNumber
	 */
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
	/**
	 * SQL query for inserting the right space in the nested set tree for a module which is connected to another module. 
	 * Input: parentModuleManufacturer, parentModuleTypeNumber, parentModuleSerialNumber
	 */
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
	/**
	 * SQL query for removing a module. 
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeModule =
			"DELETE FROM Module \n" + 
			"WHERE manufacturer = ? AND \n" +  
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;"; 
	/**
	 * SQL query for removing the left space of a module in the nested set tree.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeSpaceInNestedTreeForModuleLeft =
			"UPDATE Module \n" + 
			"SET attachedToLeft = attachedToLeft - 2 \n" +  
			"WHERE attachedToLeft >= ( \n" + 
			"	SELECT attachedToRight \n" + 
			"	FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			");"; 
	/**
	 * SQL query for removing the right space of a module in the nested set tree.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeSpaceInNestedTreeForModuleRight =
			"UPDATE Module \n" + 
			"SET attachedToRight = attachedToRight - 2 \n" +  
			"WHERE attachedToRight >= ( \n" + 
			"	SELECT attachedToRight \n" + 
			"	FROM (SELECT * FROM Module) AS tbl1 \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" + 
			");"; 
	
	/**
	 * The {@link ModuleListener} that is passed to the instantiated modules.
	 */
	private ModuleListener moduleListener;
	/**
	 * The {@link DynamicClassFactory} used by the CapabilityFactory to load classes of capabilities.
	 */
	private DynamicClassFactory<Module> dynamicClassFactory;
	private HardwareAbstractionLayer hal;
	
	private HashMap<ModuleIdentifier, Module> loadedModules;

	/**
	 * Constructs a new ModuleFactory with a new {@link KnowledgeDBClient}.
	 * @param hal
	 * @throws KnowledgeException
	 */
	public ModuleFactory(ModuleListener moduleListener, HardwareAbstractionLayer hal) throws KnowledgeException{
		super(new KnowledgeDBClient());
		this.moduleListener = moduleListener;
		this.dynamicClassFactory = new DynamicClassFactory<>();
		this.hal = hal;
		this.loadedModules = new HashMap<ModuleIdentifier, Module>();
	}
	/**
	 * This method gets the bottomModules that match with a functionalModuleTree of a capability.
	 * @param capability
	 * @param treeNumber
	 * @return
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	public ArrayList<ModuleActor> getBottomModulesForFunctionalModuleTree(Capability capability, int treeNumber) throws FactoryException, JarFileLoaderException{
		ArrayList<ModuleActor> modules = new ArrayList<ModuleActor>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersOfPhysicalModuleTreesForFunctionalModuleTreeOfACapabilityType, 
					capability.getName(), treeNumber, hal.getEquipletName());
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
		System.out.println(modules);
		return modules;
	}
	/**
	 * This method gets all the bottomModules of the equiplet.
	 * @return
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	public ArrayList<Module> getBottomModules() throws FactoryException, JarFileLoaderException {
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
	/**
	 * This method executes the {@link HardwareStep} by instantiating the module and forwarding the HardwareStep to it.
	 * @param processListener
	 * @param hardwareStep
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 * @throws ModuleExecutingException
	 */
	public void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws FactoryException, JarFileLoaderException, ModuleExecutingException{
		ModuleActor module = (ModuleActor) getModuleByIdentifier(hardwareStep.getModuleIdentifier());
		module.executeHardwareStep(processListener, hardwareStep);
	}
	
	/**
	 * This method will return the instantiated module for the {@link ModuleIdentifier}.
	 * If the module has not been instantiated, it will be instantiated by downloading the software from the knowledge database and dynamically loading the class.
	 * @param moduleIdentifier
	 * @return
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	public Module getModuleByIdentifier(ModuleIdentifier moduleIdentifier) throws FactoryException, JarFileLoaderException{
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
	/**
	 * This method determines if a moduleType (identified by the {@link ModuleIdentifier}) is known in the knowledge database.
	 * @param moduleIdentifier
	 * @return true if the moduleType is known in the knowledge database, false otherwise.
	 */
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
	
	/**
	 * This methods attempts to insert a module in the database using the data provided in the JsonObjects.
	 * @param staticSettings contains all the static information of the module (software, properties, calibrationData, etc).
	 * @param dynamicSettings contains all the dynamic information of the module (mount position, attached to other modules, orientation, etc).
	 * @return true if insertion of the module is successful, false otherwise. 
	 */
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
				deserializeModuleCalibrationData(calibrationEntries);
				
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
	/**
	 * This methods attempts to update a module in the database using the data provided in the JsonObjects.
	 * @param staticSettings contains all the static information of the module (software, properties, calibrationData, etc).
	 * @return true if insertion of the module is successful, false otherwise. 
	 */
	public boolean updateModule(JsonObject staticSettings,
			JsonObject dynamicSettings) {
		// TODO Auto-generated method stub
		return false;
	}
	/**
	 * This method removes a module from the knowledge database.
	 * @param moduleIdentifier
	 * @return the static information of the module.
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	public JsonObject removeModule(ModuleIdentifier moduleIdentifier) throws FactoryException, JarFileLoaderException{
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
			type.add("rosSoftware", rosSoftware.serialize());
			
			type.add("supportedMutations", Mutation.serializeAllSupportedMutations(moduleIdentifier, knowledgeDBClient));
			Mutation.removeSupportedMutations(moduleIdentifier, knowledgeDBClient);
			
			output.add("calibrationData", serializeModuleCalibrationData(moduleIdentifier));
			knowledgeDBClient.executeUpdateQuery(removeAllCalibrationDataForModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			
			Row[] moduleTypeRows = knowledgeDBClient.executeSelectQuery(getModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			type.addProperty("properties", (String) moduleTypeRows[0].get("moduleTypeProperties"));
			
			Row[] moduleRows = knowledgeDBClient.executeSelectQuery(getModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			output.addProperty("properties", (String) moduleRows[0].get("moduleProperties"));
			
			removeSpace(moduleIdentifier);
			knowledgeDBClient.executeUpdateQuery(removeModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			knowledgeDBClient.executeUpdateQuery(removeModuleTypesWithNoModules);
			
			output.add("type", type);
			return output;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			throw new FactoryException("deletion of module failed :(", ex);
		}
	}
	
	/**
	 * This method inserts space in the nested set tree for a module to be attached to the parentModule.
	 * @param parentModuleIdentifier
	 * @throws KnowledgeException
	 */
	private void insertSpace(ModuleIdentifier parentModuleIdentifier) throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleLeft, parentModuleIdentifier.getManufacturer(), 
				parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleRight, parentModuleIdentifier.getManufacturer(), 
				parentModuleIdentifier.getTypeNumber(), parentModuleIdentifier.getSerialNumber());
	}
	/**
	 * This method removes space in the nested set tree for a module.
	 * @param moduleIdentifier is the identifier of the module to be removed. 
	 * @throws KnowledgeException
	 */
	private void removeSpace(ModuleIdentifier moduleIdentifier) throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(removeSpaceInNestedTreeForModuleLeft, moduleIdentifier.getManufacturer(), 
				moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(removeSpaceInNestedTreeForModuleRight, moduleIdentifier.getManufacturer(), 
				moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
	}
	/**
	 * This method inserts a moduleType in the knowledge database.
	 * @param moduleIdentifier
	 * @param type
	 * @return
	 * @throws KnowledgeException
	 */
	private boolean insertModuleType(ModuleIdentifier moduleIdentifier, JsonObject type) throws KnowledgeException {
		JsonObject halSoftwareObject = type.get("halSoftware").getAsJsonObject();
		JavaSoftware halSoftware = JavaSoftware.insertJavaSoftware(halSoftwareObject, knowledgeDBClient);
		int halSoftwareId = halSoftware.getId();
		
		Integer rosSoftwareId = null;
		if(type.get("rosSoftware").isJsonNull() == false) {
			JsonObject rosSoftwareObject = type.get("rosSoftware").getAsJsonObject();
			RosSoftware rosSoftware = RosSoftware.insertRosSoftware(rosSoftwareObject, knowledgeDBClient);
			rosSoftwareId = rosSoftware.getId();
		}
		
		String properties = type.get("properties").getAsString();
		knowledgeDBClient.executeUpdateQuery(addModuleType, moduleIdentifier.getManufacturer(), 
				moduleIdentifier.getTypeNumber(), properties, halSoftwareId, rosSoftwareId);
		
		JsonArray supportedMutationEntries = type.get("supportedMutations").getAsJsonArray();
		Mutation.insertSupportedMutations(moduleIdentifier, supportedMutationEntries, knowledgeDBClient);
		
		return true;
	}
	/**
	 * This method updates a moduleType in the knowledge database.
	 * It will update the software of the moduleType if the buildNumber of the provided software is higher than the buildNumber of the currently stored software.
	 * @param moduleIdentifier
	 * @param type
	 */
	private void updateModuleType(ModuleIdentifier moduleIdentifier, JsonObject type) {
		JsonObject halSoftwareObject = type.get("halSoftware").getAsJsonObject();
		JavaSoftware javaSoftware = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		int currentJavaSoftwareBuildNumber = javaSoftware.getBuildNumber();
		
		int newJavaSoftwareBuildNumber = JavaSoftware.getBuildNumber(halSoftwareObject);
		
		if(newJavaSoftwareBuildNumber > currentJavaSoftwareBuildNumber) {
			// update the halSoftware
			javaSoftware.updateJavaSoftware(halSoftwareObject);
		}
		System.out.println(type.toString());
		JsonObject rosSoftwareObject = type.get("rosSoftware").getAsJsonObject();
		RosSoftware rosSoftware = RosSoftware.getRosSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		int currentRosSoftwareBuildNumber = rosSoftware.getBuildNumber();
		
		int newRosSoftwareBuildNumber = RosSoftware.getBuildNumber(rosSoftwareObject);
		
		if(newRosSoftwareBuildNumber > currentRosSoftwareBuildNumber) {
			// update the halSoftware
			rosSoftware.updateRosSoftware(rosSoftwareObject);
		}
		
	}
	
	/**
	 * This method will serialize all the moduleCalibrationData associated with the module identified by the {@link ModuleIdentifier}.
	 * This metohd will NOT remove the serialized moduleCalibrationData. 
	 * @param moduleIdentifier
	 * @return
	 */
	private JsonArray serializeModuleCalibrationData(ModuleIdentifier moduleIdentifier) {
		JsonArray calibrationEntries = new JsonArray();
		try {
			Row[] calibrationDataRows = knowledgeDBClient.executeSelectQuery(getAllModuleCalibrationDataForModule, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), moduleIdentifier.getSerialNumber());
			for (Row calibrationDataRow : calibrationDataRows) {
				Integer moduleCalibrationId = (Integer) calibrationDataRow.get("id");
				String dateTime = ((Timestamp) calibrationDataRow.get("date")).toString();
				String properties = (String) calibrationDataRow.get("properties");
				
				JsonObject calibrationDataEntry = new JsonObject();
				calibrationDataEntry.addProperty("date", dateTime);
				calibrationDataEntry.addProperty("data", properties);
				
				// fetch the moduleSet for the calibration data
				JsonArray moduleEntries = new JsonArray();
				Row[] moduleSetrows = knowledgeDBClient.executeSelectQuery(getModuleSetForModuleCalibrationData, moduleCalibrationId);
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
	/**
	 * This method will deserialize all the moduleCalibration data provided and store it in the knowledge database.
	 * @param moduleCalibrationEntries
	 */
	private void deserializeModuleCalibrationData(JsonArray moduleCalibrationEntries) {
		try {
			for (JsonElement moduleCalibrationEntryElement : moduleCalibrationEntries) {
				JsonObject moduleCalibrationEntry = moduleCalibrationEntryElement.getAsJsonObject();
				String dateTime = moduleCalibrationEntry.get("date").getAsString();
				String properties = moduleCalibrationEntry.get("data").getAsString();
				int calibrationDataId = knowledgeDBClient.executeUpdateQuery(addModuleCalibrationData, dateTime, properties);
				
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
	
	public HardwareAbstractionLayer getHAL() {
		return hal;
	}
}