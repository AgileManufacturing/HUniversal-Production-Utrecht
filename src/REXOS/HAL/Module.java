package HAL;

import HAL.exceptions.FactoryException;
import HAL.factories.ModuleFactory;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;
import HAL.listeners.BlackboardModuleListener;
import HAL.listeners.ModuleListener;
/**
 * Abstract representation of a module in the HAL.
 * @author Bas Voskuijlen
 *
 */
public abstract class Module implements BlackboardModuleListener { 
	protected KnowledgeDBClient knowledgeDBClient;
	protected ModuleIdentifier moduleIdentifier;
	protected ModuleFactory moduleFactory;
	protected ModuleListener moduleListener;
	
	private static final String GET_MOUNT_POSITION = 
			"SELECT mountPointX, mountPointY FROM Module " +
			"	WHERE manufacturer = ?" +
			" 		AND typeNumber = ?" +
			" 		AND serialNumber = ?";
	private static final String GET_PARENT_MODULE =
			"SELECT * FROM Module " +
			"	WHERE attachedToLeft < (" +
			"		SELECT attachedToLeft FROM Module " +
			"			WHERE manufacturer = ?" +
			"				AND typeNumber = ?" +
			"				AND serialNumber = ?" +
			"	) AND attachedToRight > (" +
 			"		SELECT attachedToRight FROM Module " +
 			"			WHERE manufacturer = ?" +
 			"				AND typeNumber = ?" +
 			"				AND serialNumber = ?" +
 			"	) " +
 			"	ORDER BY abs(attachedToLeft - attachedToRight) ASC LIMIT 1";
	
	/**
	 * Constructs a new Module and subscribes to the blackboardHandler.
	 * @param moduleIdentifier
	 * @param moduleFactory
	 * @param moduleListener
	 * @throws KnowledgeException
	 */
	public Module(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) 
			throws KnowledgeException{
		this.moduleIdentifier = moduleIdentifier;
		this.knowledgeDBClient = new KnowledgeDBClient();
		this.moduleFactory = moduleFactory;
		this.moduleListener = moduleListener;
		
		moduleFactory.getHAL().getBlackBoardHandler().addBlackboardModuleListener(this);
	}	
	
	public ModuleIdentifier getModuleIdentifier(){
		return this.moduleIdentifier;
	}
	
	/**
	 * This method will return the mount position of the module on the mountplate of the equiplet.
	 * @see http://wiki.agilemanufacturing.nl/index.php/Coordinate_systems
	 * @return An integer array with 2 elements containing the zero-indexed x and y position on the mountplate (corresponding with the x and the -z axis in the equiplet coordinate system).
	 */
	public int[] getMountPosition() {
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	GET_MOUNT_POSITION, 
																moduleIdentifier.getManufacturer(), 
																moduleIdentifier.getTypeNumber(), 
																moduleIdentifier.getSerialNumber());
		if (resultSet.length == 1){
			int[] position = new int[2];
			position[0] = (int) resultSet[0].get("mountPointX");
			position[1] = (int) resultSet[0].get("mountPointY");
			return position;
		}
		return null;
	}
	
	/**
	 * This method will return the parent module of this module
	 * @return The parent module or null if no parent module exists
	 * @throws FactoryException
	 */
	public Module getParentModule() throws FactoryException {		
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	GET_PARENT_MODULE,
																moduleIdentifier.getManufacturer(),
																moduleIdentifier.getTypeNumber(),
																moduleIdentifier.getSerialNumber(),
																moduleIdentifier.getManufacturer(),
																moduleIdentifier.getTypeNumber(),
																moduleIdentifier.getSerialNumber());
		
		if (resultSet.length == 1){
			ModuleIdentifier moduleIdentifier = new ModuleIdentifier(
					resultSet[0].get("manufacturer").toString(),
					resultSet[0].get("typeNumber").toString(),
					resultSet[0].get("serialNumber").toString());
			return this.moduleFactory.getModuleByIdentifier(moduleIdentifier);
		}
		else return null;
	}

	/**
	 * This method will return the properties of this specific module
	 * @return
	 */
	public String getProperties() {
		// TODO Auto-generated method stub
		return null;
	}
}
