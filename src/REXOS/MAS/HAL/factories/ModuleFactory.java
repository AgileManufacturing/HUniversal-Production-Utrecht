package HAL;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

import com.google.gson.JsonObject;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.DynamicClassFactory;
import libraries.dynamicloader.InstantiateClassException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;
import HAL.listeners.ModuleListener;

public class ModuleFactory extends Factory {
	// SQL queries
	private static final String getModuleIdentifiersForCapability = 
			"SELECT * \n" + 
			"FROM CapabilityType +\n" + 
			"WHERE name = ? AND +\n" +
			"	NOT EXISTS( +\n" +
			"		SELECT * +\n" +
			"		FROM CapabilityTypeDependencySet +\n" +
			"		WHERE CapabilityType.name =CapabilityTypeDependencySet.capabilityType AND +\n" +
			"		treeNumber NOT IN( +\n" +
			"			SELECT treeNumber +\n" +
			"			FROM CapabilityTypeDependencySet AS currentDependencySet +\n" +
			"			JOIN Module AS currentModule +\n" +
			"			WHERE CapabilityType.name = currentDependencySet.capabilityType AND +\n" +
			"			NOT EXISTS( +\n" +
			"				SELECT * +\n" +
			"				FROM CapabilityTypeDependencySet +\n" +
			"				WHERE currentDependencySet.capabilityType = capabilityType AND +\n" +
			"				currentDependencySet.treeNumber = treeNumber AND +\n" +
			"				commandType NOT IN( +\n" +
			"					SELECTcommandType +\n" +
			"					FROM ModuleCommandType +\n" +
			"					JOIN Module ON ModuleCommandType.manufacturer = Module.manufacturer AND +\n" +
			"						ModuleCommandType.typeNumber = Module.typeNumber +\n" +
			"					WHERE currentModule.attachedToLeft >= attachedToLeft AND +\n" +
			"						currentModule.attachedToRight <= attachedToRight +\n" +
			"				) +\n" +
			"			) AND +\n" +
			"			currentModule.attachedToRight =currentModule.attachedToLeft + 1 +\n" +
			"		) +\n" +
			"	);";
	private static final String getModuleIdentifiersForBotomModules = 
			"SELECT attachedToLeft, attachedToRight, Equiplet \n" + 
			"FROM Module" + 
			"WHERE currentModule.Equiplet = ? AND" + 
			"	attachedToRight = attachedToLeft + 1;"; 
	
	private KnowledgeDBClient knowledgeDBClient;
	private DynamicClassFactory<Module> dynamicClassFactory;
	
	public ModuleFactory(ModuleListener moduleListener) throws KnowledgeException{
		this.knowledgeDBClient = new KnowledgeDBClient();
		this.dynamicClassFactory = new DynamicClassFactory<>(this);
	}
	public ArrayList<Module> getBottomModulesForCapability(Capability capability) throws Exception{
		ArrayList<Module> modules = new ArrayList<Module>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleIdentifiersForCapability, capability.getName());
			for (Row row : rows) {
				String manufacturer = (String) row.get("manufacturer");
				String typeNumber = (String) row.get("typeNumber");
				String serialNumber = (String) row.get("serialNumber");
				ModuleIdentifier identifier = new ModuleIdentifier(manufacturer, typeNumber, serialNumber);
				modules.add(this.getModuleByIdentifier(identifier));
			}
		} catch (KnowledgeException ex) {
			throw new ModuleFactoryException("Capabilit does not exist in the knowledge database");
		} catch (KeyNotFoundException ex) {
			System.out.println("HAL::ModuleFactory::getBottomModules(): Error occured which is considered to be impossible " + ex);
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
		} catch (KnowledgeException ex) {
			// this is impossible!
			System.out.println("HAL::ModuleFactory::getBottomModules(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		} catch (KeyNotFoundException ex) {
			// this is impossible!
			System.out.println("HAL::ModuleFactory::getBottomModules(): Error occured which is considered to be impossible " + ex);
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
	public boolean insertModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings, JsonObject staticSettings){
		return true;
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings){
		return true;
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier){
		return null;
	}
}
