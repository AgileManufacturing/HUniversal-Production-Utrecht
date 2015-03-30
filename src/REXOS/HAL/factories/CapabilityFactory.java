package HAL.factories;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;

import util.log.LogSection;
import HAL.Capability;
import HAL.HardwareAbstractionLayer;
import HAL.dataTypes.JavaSoftware;
import HAL.libraries.dynamicloader.DynamicClassFactory;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

/**
 * The CapabilityFactory is the factory for the {@link Capability}.
 * It does not only instantiate capabilities using the {@link DynamicClassFactory} (allowing dynamic addition of classes) but also manages part of the knowledge database.
 * 
 * @author Tommas Bakker
 * @author Niek Arends
 *
 */
public class CapabilityFactory extends Factory<String, Capability> {
	// SQL queries

	/**
	 * SQL query for selecting the supported capabilityTypes for an equiplet.
	 * Input: equipletName
	 * A capabilityType is supported when all the functionalModuleTrees could be matched with the corresponding physicalModuleTrees.
	 */
	private static final String getSupportedCapabilityTypes = 
			"SELECT name \n" +
			"FROM CapabilityType \n" +
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM CapabilityTypeRequiredMutation \n" +
			"	WHERE CapabilityType.name = CapabilityTypeRequiredMutation.capabilityType AND \n" +
			"	treeNumber NOT IN( \n" +
			"		SELECT treeNumber \n" +
			"		FROM CapabilityTypeRequiredMutation AS currentRequiredMutation \n" +
			"		JOIN Module AS currentModule \n" +
			"		WHERE CapabilityType.name = currentRequiredMutation.capabilityType AND \n" +
			"		NOT EXISTS( \n" +
			"			SELECT * \n" +
			"			FROM CapabilityTypeRequiredMutation \n" +
			"			WHERE currentRequiredMutation.capabilityType = capabilityType AND \n" +
			"			currentRequiredMutation.treeNumber = treeNumber AND \n" +
			"			mutation NOT IN( \n" +
			"				SELECT mutation \n" +
			"				FROM SupportedMutation \n" +
			"				JOIN Module ON SupportedMutation.manufacturer = Module.manufacturer AND \n" +
			"					SupportedMutation.typeNumber = Module.typeNumber \n" +
			"				WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" +
			"					currentModule.attachedToRight <= attachedToRight AND \n" +
			"					currentModule.equiplet = ? \n" +
			"			) \n" +
			"		) AND \n" +
			"		currentModule.attachedToRight = currentModule.attachedToLeft + 1 \n" +
			"	) \n" +
			");";
	private static final String getCapabilities = 
			"SELECT name \n" +
			"FROM CapabilityType;"; 
	/**
	 * SQL query for selecting the supported capabilityTypes for a serviceType.
	 * Input: serviceTypeName, equipletName
	 * A serviceType is supported when at least one capabilityType is supported.
	 * A capabilityType is supported when all the function module trees could be matched with the corresponding physical module trees.
	 */
	private static final String getSupportedCapabilityTypesForServiceType = 
			"SELECT * \n" +
			"FROM CapabilityType \n" +
			"WHERE \n" +
			"	name IN(\n" +
			"		SELECT CapabilityType \n" +
			"		FROM ServiceType_CapabilityType \n" +
			"		WHERE ServiceType = ? \n" +
			"	) AND NOT EXISTS( \n" +
			"		SELECT * \n" +
			"		FROM CapabilityTypeRequiredMutation \n" +
			"		WHERE CapabilityType.name = CapabilityTypeRequiredMutation.capabilityType AND \n" +
			"		treeNumber NOT IN( \n" +
			"			SELECT treeNumber \n" +
			"			FROM CapabilityTypeRequiredMutation AS currentRequiredMutation \n" +
			"			JOIN Module AS currentModule \n" +
			"			WHERE CapabilityType.name = currentRequiredMutation.capabilityType AND \n" +
			"			NOT EXISTS( \n" +
			"				SELECT * \n" +
			"				FROM CapabilityTypeRequiredMutation \n" +
			"				WHERE currentRequiredMutation.capabilityType = capabilityType AND \n" +
			"				currentRequiredMutation.treeNumber = treeNumber AND \n" +
			"				mutation NOT IN( \n" +
			"					SELECT mutation \n" +
			"					FROM SupportedMutation \n" +
			"					JOIN Module ON SupportedMutation.manufacturer = Module.manufacturer AND \n" +
			"						SupportedMutation.typeNumber = Module.typeNumber \n" +
			"					WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" +
			"						currentModule.attachedToRight <= attachedToRight AND \n" +
			"						currentModule.equiplet = ? \n" +
			"				) \n" +
			"			) AND \n" +
			"			currentModule.attachedToRight = currentModule.attachedToLeft + 1 \n" +
			"		) \n" +
			"	);";
	
	/**
	 * The {@link DynamicClassFactory} used by the CapabilityFactory to load classes of capabilities.
	 */
	private HardwareAbstractionLayer hal;
	/**
	 * Constructs a new CapabilityFactory with a new {@link KnowledgeDBClient}.
	 * 
	 * @param hal
	 * @throws KnowledgeException
	 */
	public CapabilityFactory(HardwareAbstractionLayer hal) throws KnowledgeException {
		super(new KnowledgeDBClient());
		this.hal = hal;
	}
	/**
	 * This method will all return all the capabilities supported by this equiplet.
	 * 
	 * @return
	 * @throws Exception
	 */
	public ArrayList<Capability> getAllSupportedCapabilities() throws Exception {
		ArrayList<Capability> capabilities = new ArrayList<Capability>();

		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilityTypes, hal.getEquipletName());
		for (Row row : rows) {
			String capabilityName = (String) row.get("name");
			capabilities.add(this.getItemForIdentifier(capabilityName));
		}
		return capabilities;
	}
	/**
	 * This method will all return all the capabilities associated with the service and supported by this equiplet.
	 * 
	 * @param service
	 * @return
	 * @throws Exception
	 */
	public ArrayList<Capability> getCapabilitiesForService(String service) {
		ArrayList<Capability> capabilities = new ArrayList<Capability>();

		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilityTypesForServiceType, service, hal.getEquipletName());
		for (Row row : rows) {
			String capabilityName = (String) row.get("name");
			capabilities.add(this.getItemForIdentifier(capabilityName));
		}
		return capabilities;
	}
	
	/**
	 * This moethod will return all the capabilities.
	 */
	public ArrayList<String> getCapabilities() {
		ArrayList<String> capabilities = new ArrayList<String>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getCapabilities);
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getCapabilities", rows);
		for (Row row : rows) {
			capabilities.add((String) row.get("name"));
		}
		
		return capabilities;
	}
	
	@Override
	protected JavaSoftware getJavaSoftware(String identifier) {
		return JavaSoftware.getJavaSoftwareForCapabilityName(identifier);
	}
	@Override
	protected Capability getConstuctorforThisFactory(Class<Capability> myClass, String key) throws NoSuchMethodException, SecurityException, InstantiationException,
			IllegalAccessException, IllegalArgumentException, InvocationTargetException {
		return myClass.getConstructor(ModuleFactory.class).newInstance(hal.getModuleFactory());
	}
	
	@Override
	protected ArrayList<String> getKeysToKeepInCache() {
		return getCapabilities();
	}
}
