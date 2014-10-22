package HAL.factories;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.sql.SQLException;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Capability;
import HAL.HardwareAbstractionLayer;
import HAL.JavaSoftware;
import HAL.Module;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.ModuleTypeIdentifier;
import HAL.Mutation;
import HAL.RosSoftware;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleExecutingException;
import HAL.libraries.dynamicloader.DynamicClassDescription;
import HAL.libraries.dynamicloader.DynamicClassFactory;
import HAL.libraries.dynamicloader.InstantiateClassException;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * The ModuleFactory is the factory for the {@link Module}s. 
 * It does not only instantiate capabilities using the {@link DynamicClassFactory} (allowing dynamic addition of classes) but also manages part of the knowledge database.
 * @author Tommas Bakker
 * @author Niek Arends
 *
 */
public class ModuleFactory extends Factory<ModuleIdentifier, Module> {
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
	 * The {@link ModuleListener} that is passed to the instantiated modules.
	 */
	private ModuleListener moduleListener;
	/**
	 * The {@link DynamicClassFactory} used by the CapabilityFactory to load classes of capabilities.
	 */
	private HardwareAbstractionLayer hal;
	
	/**
	 * Constructs a new ModuleFactory with a new {@link KnowledgeDBClient}.
	 * @param hal
	 * @throws KnowledgeException
	 */
	public ModuleFactory(ModuleListener moduleListener, HardwareAbstractionLayer hal) throws KnowledgeException{
		super(new KnowledgeDBClient());
		this.moduleListener = moduleListener;
		this.hal = hal;
	}
	/**
	 * This method gets the bottomModules that match with a functionalModuleTree of a capability.
	 * @param capability
	 * @param treeNumber
	 * @return
	 * @throws FactoryException
	 */
	public ArrayList<ModuleActor> getBottomModulesForFunctionalModuleTree(Capability capability, int treeNumber){
		ArrayList<ModuleActor> modules = new ArrayList<ModuleActor>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersOfPhysicalModuleTreesForFunctionalModuleTreeOfACapabilityType, 
				capability.getName(), treeNumber, hal.getEquipletName());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getModuleIdentifiersOfPhysicalModuleTreesForFunctionalModuleTreeOfACapabilityType", rows);
		for (Row row : rows) {
			String manufacturer = (String) row.get("manufacturer");
			String typeNumber = (String) row.get("typeNumber");
			String serialNumber = (String) row.get("serialNumber");
			ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
			modules.add((ModuleActor) this.getSomethingByIdentifier(identifier));
		}
		Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.DEBUG, "Found bottomModules for function module tree " + treeNumber + " of capability " + capability.getName() + ":", 
				modules);
		return modules;
	}
	/**
	 * This method gets all the bottomModules of the equiplet.
	 * @return
	 * @throws FactoryException
	 */
	public ArrayList<Module> getBottomModules() {
		ArrayList<Module> modules = new ArrayList<Module>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersForBotomModules, hal.getEquipletName());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getModuleIdentifiersForBotomModules", rows);
		for (Row row : rows) {
			String manufacturer = (String) row.get("manufacturer");
			String typeNumber = (String) row.get("typeNumber");
			String serialNumber = (String) row.get("serialNumber");
			
			ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
			modules.add((ModuleActor) this.getSomethingByIdentifier(identifier));
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
	public void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws 
			FactoryException, JarFileLoaderException {
		ModuleActor module;
		module = (ModuleActor) this.getSomethingByIdentifier(hardwareStep.getModuleIdentifier());
		module.executeHardwareStep(processListener, hardwareStep);	
	}
	
	protected JavaSoftware getJavaSoftware(ModuleIdentifier key) {
		return JavaSoftware.getJavaSoftwareForModuleIdentifier(key);
	}
	
	
	public HardwareAbstractionLayer getHAL() {
		return hal;
	}
	@Override
	protected Module getConstuctorforThisFactory(Class<Module> myClass, ModuleIdentifier key) throws NoSuchMethodException,
			SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
		return myClass.getConstructor(ModuleIdentifier.class ,ModuleFactory.class ,ModuleListener.class).newInstance(key, this, moduleListener);
	}
	
	public void removeModulesOfTypeFromCache(ModuleTypeIdentifier type) {
		ArrayList<ModuleIdentifier> modulesToRemove = new ArrayList<ModuleIdentifier>();
		for (ModuleIdentifier key : cache.keySet()) {
			if(type.equals(key) == true) modulesToRemove.add(key);
		}
		for (ModuleIdentifier moduleIdentifier : modulesToRemove) {
			removeItemFromCache(moduleIdentifier);
		}
	}
}