package HAL.factories;

import java.util.ArrayList;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.DynamicClassFactory;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;
import HAL.Capability;
import HAL.HardwareAbstractionLayer;
import HAL.Service;

public class CapabilityFactory extends Factory{
	// SQL queries
	private static final String getSupportedCapabilities = 
			"SELECT * \n" + 
			"FROM CapabilityType \n" + 
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM CapabilityTypeDependencySet \n" +
			"	WHERE CapabilityType.name =CapabilityTypeDependencySet.capabilityType AND \n" +
			"	treeNumber NOT IN( \n" +
			"		SELECT treeNumber \n" +
			"		FROM CapabilityTypeDependencySet AS currentDependencySet \n" +
			"		JOIN Module AS currentModule \n" +
			"		WHERE CapabilityType.name = currentDependencySet.capabilityType AND \n" +
			"		NOT EXISTS( \n" +
			"			SELECT * \n" +
			"			FROM CapabilityTypeDependencySet \n" +
			"			WHERE currentDependencySet.capabilityType = capabilityType AND \n" +
			"			currentDependencySet.treeNumber = treeNumber AND \n" +
			"			commandType NOT IN( \n" +
			"				SELECT commandType \n" +
			"				FROM ModuleCommandType \n" +
			"				JOIN Module ON ModuleCommandType.manufacturer = Module.manufacturer AND \n" +
			"					ModuleCommandType.typeNumber = Module.typeNumber \n" +
			"				WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" +
			"					currentModule.attachedToRight <= attachedToRight AND \n" +
			"					currentModule.equiplet = ? \n" +
			"			) \n" +
			"		) AND \n" +
			"		currentModule.attachedToRight = currentModule.attachedToLeft + 1 \n" +
			"	) \n" +
			");";
	private static final String getSupportedCapabilitiesForService = 
			"SELECT * \n" + 
			"FROM CapabilityType \n" + 
			"WHERE \n" +
			"	name IN(\n" +
			"		SELECT * CapabilityType\n" +
			"		FROM ServiceType_CapabilityType\n" +
			"		WHERE ServiceType = ?\n" +
			"	) AND NOT EXISTS( \n" +
			"		SELECT * \n" +
			"		FROM CapabilityTypeDependencySet \n" +
			"		WHERE CapabilityType.name =CapabilityTypeDependencySet.capabilityType AND \n" +
			"		treeNumber NOT IN( \n" +
			"			SELECT treeNumber \n" +
			"			FROM CapabilityTypeDependencySet AS currentDependencySet \n" +
			"			JOIN Module AS currentModule \n" +
			"			WHERE CapabilityType.name = currentDependencySet.capabilityType AND \n" +
			"			NOT EXISTS( \n" +
			"				SELECT * \n" +
			"				FROM CapabilityTypeDependencySet \n" +
			"				WHERE currentDependencySet.capabilityType = capabilityType AND \n" +
			"				currentDependencySet.treeNumber = treeNumber AND \n" +
			"				commandType NOT IN( \n" +
			"					SELECT commandType \n" +
			"					FROM ModuleCommandType \n" +
			"					JOIN Module ON ModuleCommandType.manufacturer = Module.manufacturer AND \n" +
			"						ModuleCommandType.typeNumber = Module.typeNumber \n" +
			"					WHERE currentModule.attachedToLeft >= attachedToLeft AND \n" +
			"						currentModule.attachedToRight <= attachedToRight AND \n" +
			"						currentModule.equiplet = ? \n" +
			"				) \n" +
			"			) AND \n" +
			"			currentModule.attachedToRight = currentModule.attachedToLeft + 1 \n" +
			"		) \n" +
			"	);";
	
	private DynamicClassFactory<Capability> dynamicClassFactory;
	private HardwareAbstractionLayer hal;
	
	public CapabilityFactory(HardwareAbstractionLayer hal) throws KnowledgeException {
		super(new KnowledgeDBClient());
		this.hal = hal;
	}
	
	public ArrayList<Capability> getAllSupportedCapabilities() throws Exception{
		ArrayList<Capability> capabilities = new ArrayList<Capability>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilities, hal.getEquipletName());
			for (Row row : rows) {
				String capabilityName = (String) row.get("name");
				capabilities.add(this.getCapabilityByName(capabilityName));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::CapabilityFactory::getAllSupportedCapabilities(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
		
		return capabilities;
	}
	public ArrayList<Capability> getCapabilitiesForService(Service service) throws Exception{
		ArrayList<Capability> capabilities = new ArrayList<Capability>();
		
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedCapabilitiesForService, service.getName(), hal.getEquipletName());
			for (Row row : rows) {
				String capabilityName = (String) row.get("name");
				capabilities.add(this.getCapabilityByName(capabilityName));
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::CapabilityFactory::getCapabilitiesForService(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
		
		return capabilities;
	}
	private Capability getCapabilityByName(String capabilityName) throws Exception {
		DynamicClassDescription description = new DynamicClassDescription(1, "a");
		Class<Capability> capabilityClass = dynamicClassFactory.getClassFromDescription(description);
		return capabilityClass.getConstructor().newInstance();
	}
}
