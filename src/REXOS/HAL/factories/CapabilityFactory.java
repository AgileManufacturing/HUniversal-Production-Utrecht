package HAL.factories;

import generic.Service;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.HashMap;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Capability;
import HAL.HardwareAbstractionLayer;
import HAL.JavaSoftware;
import HAL.Module;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.libraries.dynamicloader.DynamicClassDescription;
import HAL.libraries.dynamicloader.DynamicClassFactory;
import HAL.libraries.dynamicloader.InstantiateClassException;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;
import HAL.listeners.ModuleListener;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * The CapabilityFactory is the factory for the {@link Capability}. 
 * It does not only instantiate capabilities using the {@link DynamicClassFactory} (allowing dynamic addition of classes) but also manages part of the knowledge database.
 * @author Tommas Bakker
 * @author Niek Arends
 *
 */
public class CapabilityFactory extends Factory<String, Capability>{
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
	 * SQL query for selecting the serviceTypes for a capabilityType.
	 * Input: capabilityTypeName
	 */
	public static final String getServiceTypesForCapabilityType =
			"SELECT serviceType \n" + 
			"FROM ServiceType_CapabilityType \n" + 
			"WHERE capabilityType = ?;";
	
	/**
	 * The {@link DynamicClassFactory} used by the CapabilityFactory to load classes of capabilities.
	 */
	private DynamicClassFactory<Capability> dynamicClassFactory;
	private HardwareAbstractionLayer hal;
	private HashMap<String, Capability> loadedCapabilities;
	/**
	 * Constructs a new CapabilityFactory with a new {@link KnowledgeDBClient}.
	 * @param hal
	 * @throws KnowledgeException
	 */
	public CapabilityFactory(HardwareAbstractionLayer hal) throws KnowledgeException {
		super(new KnowledgeDBClient(), hal);
		this.hal = hal;
		this.dynamicClassFactory = new DynamicClassFactory<>();
		this.loadedCapabilities = new HashMap<String, Capability>();
	}
	
	/**
	 * This method will all return all the capabilities supported by this equiplet.
	 * @return
	 * @throws Exception
	 */
	public ArrayList<Capability> getAllSupportedCapabilities() throws Exception{
		ArrayList<Capability> capabilities = new ArrayList<Capability>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilityTypes, hal.getEquipletName());
		for (Row row : rows) {
			String capabilityName = (String) row.get("name");
			capabilities.add(this.<String, Capability>getSomethingByIdentifier(capabilityName));
		}
		return capabilities;
	}

	/**
	 * This method will all return all the capabilities associated with the service and supported by this equiplet. 
	 * @param service
	 * @return
	 * @throws Exception
	 */
	public ArrayList<Capability> getCapabilitiesForService(Service service) {
		ArrayList<Capability> capabilities = new ArrayList<Capability>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilityTypesForServiceType, service.getName(), hal.getEquipletName());
		for (Row row : rows) {
			String capabilityName = (String) row.get("name");
			try{
				capabilities.add(this.getSomethingByIdentifier(capabilityName));
			} catch (FactoryException | JarFileLoaderException ex) {
				Logger.log(LogSection.HAL_CAPABILITY_FACTORY, LogLevel.ERROR, "Unable to get capability with name: " + capabilityName, ex);
			}
		}
		return capabilities;
	}

	protected JavaSoftware getJavaSoftware(String identifier) {
		// TODO Auto-generated method stub
		return JavaSoftware.getJavaSoftwareForCapabilityIdentifier(identifier);
	}

	@Override
	protected Capability getConstuctorforThisFactory(Class<Capability> myClass, String key) throws NoSuchMethodException,	SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
		// TODO Auto-generated method stub
		return myClass.getConstructor(ModuleFactory.class).newInstance(hal.getModuleFactory());
	}
}
