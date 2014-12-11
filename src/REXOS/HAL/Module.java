package HAL;

import java.util.Vector;

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
 * @author Lars Veenendaal
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
 	private static final String GET_MOUNT_POINT_X =
		"SELECT mountPointX FROM Module " +
		"	WHERE manufacturer = ?" +
		"		AND typeNumber = ?" +
		"		AND serialNumber = ?;";
	private static final String GET_MOUNT_POINT_Y =
		"SELECT mountPointY FROM Module " +
		"	WHERE manufacturer = ?" +
		"		AND typeNumber = ?" +
		"		AND serialNumber = ?;";
	private static final String SET_MOUNT_POINT_X =
		"UPDATE Module " +
		"	SET mountPointX = ?" +
		"		WHERE manufacturer = ? AND " +
		"			typeNumber = ? AND " +
		"			serialNumber = ?;";
	private static final String SET_MOUNT_POINT_Y =
		"UPDATE Module " +
		"	SET mountPointY = ?" +
		"		WHERE manufacturer = ? AND " +
		"			typeNumber = ? AND " +
		"			serialNumber = ?;";
	private static final String GET_CALIBRATION_DATA_FOR_MODULE_ONLY = 
		"SELECT properties " +
		"	FROM ModuleCalibration " +
		"		JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id " +
		"			WHERE manufacturer = ? AND " +
		"				typeNumber = ? AND " +
		"				serialNumber = ? AND " +
		"				( " +
		"				SELECT count(*) " +
		"					FROM ModuleCalibrationModuleSet AS subTable " +
		"					WHERE ModuleCalibrationModuleSet.ModuleCalibration = subTable.ModuleCalibration " +
		") = 1;";
	private static final String GET_CALIBRATION_DATA_FOR_MODULE_AND_CHILDS = 
		"SELECT properties " +
		"	FROM ModuleCalibration " +
		"		JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id " +
		"			WHERE manufacturer = ? AND " +
		"				typeNumber = ? AND " +
		"				serialNumber = ? AND " +
		"				( " +
		"				SELECT count(*) " +
		"					FROM ModuleCalibrationModuleSet AS subTable " +
		"					WHERE ModuleCalibrationModuleSet.ModuleCalibration = subTable.ModuleCalibration " +
		") = 1;";
	private static final String GET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULES = 
		"SELECT properties " +
		"	FROM ModuleCalibration " +
		"		JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id " +
		"		WHERE manufacturer = ? AND " +
		"			typeNumber = ? AND " +
		"			serialNumber = ? AND " +
		"		( " +
		"		SELECT count(*) " +
		"			FROM ModuleCalibrationModuleSet AS subTable " +
		"			WHERE ModuleCalibrationModuleSet.ModuleCalibration = subTable.ModuleCalibration " +
		") = 1;";
	private static final String GET_CHILD_MODULES_INDENTIFIERS =
		"SELECT manufacturer, typeNumber, serialNumber " +
		"FROM Module " +
		"WHERE attachedToLeft > (" +
		"	SELECT attachedToLeft FROM Module " +
		"	WHERE manufacturer = ? " +
		"		AND typeNumber = ? " +
		"		AND serialNumber = ? " +
		") AND attachedToRight < (" +
	 	"	SELECT attachedToRight FROM Module " +
		"	WHERE manufacturer = ? " +
		"		AND typeNumber = ? " +
		"		AND serialNumber = ?);";


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
			return this.moduleFactory.getSomethingByIdentifier(moduleIdentifier);
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
	
	/**
	 * This method will forward the changed MAST module state to the {@link ModuleListener}
	 * Do not call this method!
	 */
	@Override
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	/**
	 * This method will forward the changed MAST module mode to the {@link ModuleListener}
	 * Do not call this method!
	 */
	@Override
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}

	/*
	
	11 - 12 - 14 - ROS -> HAL Additions

	std::string Module::getModuleProperties(){
	void Module::setModuleProperties(std::string jsonProperties) {
	std::vector<rexos_datatypes::ModuleIdentifier> Module::getChildModulesIdentifiers() {

	// UPDATE
	private static final String updateRosSoftware =
			"UPDATE RosSoftware \n" + 
			"SET buildNumber =? AND \n" + 
			"command = ? AND \n" + 
			"zipFile = ?;";
	public void updateRosSoftware(JSONObject rosSoftware) throws JSONException {
		byte[] zipFile = Base64.decodeBase64(rosSoftware.getString("zipFile").getBytes());
		int buildNumber = getBuildNumber(rosSoftware);
		String command = getCommand(rosSoftware);
		
		knowledgeDBClient.executeUpdateQuery(updateRosSoftware, buildNumber, command, zipFile);
	}




	// INSERT
	private static final String addRosSoftware =
			"INSERT INTO RosSoftware \n" + 
			"(buildNumber, command, zipFile) \n" + 
			"VALUES(?, ?, ?);";

	public static RosSoftware insertRosSoftware(JSONObject rosSoftware, KnowledgeDBClient knowledgeDBClient) throws JSONException {
		byte[] zipFile = Base64.decodeBase64(rosSoftware.getString("rosFile").getBytes());
		int buildNumber = getBuildNumber(rosSoftware);
		String command = getCommand(rosSoftware);
		
		int id = knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
				buildNumber, command, zipFile);
		return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
	}
	 */

	public int getMountPointX(){
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_MOUNT_POINT_X, 
			moduleIdentifier.getManufacturer(), 
			moduleIdentifier.getTypeNumber(), 
			moduleIdentifier.getSerialNumber());

		if (resultSet.length == 1){
			return (int) resultSet[0].get("mountPointX");
		}
		return 0;
	} 

	public int getMountPointY(){
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_MOUNT_POINT_Y, 
			moduleIdentifier.getManufacturer(), 
			moduleIdentifier.getTypeNumber(), 
			moduleIdentifier.getSerialNumber());

		if (resultSet.length == 1){
			return (int) resultSet[0].get("mountPointY");
		}
		return 0;
	}

	public void setMountPointX(int mountPointX){
		knowledgeDBClient.executeSelectQuery(	
			SET_MOUNT_POINT_X, 
			mountPointX,
			moduleIdentifier.getManufacturer(), 
			moduleIdentifier.getTypeNumber(), 
			moduleIdentifier.getSerialNumber());
	} 
	public void setMountPointY(int mountPointY){
		knowledgeDBClient.executeSelectQuery(	
			SET_MOUNT_POINT_Y, 
			mountPointY,
			moduleIdentifier.getManufacturer(), 
			moduleIdentifier.getTypeNumber(), 
			moduleIdentifier.getSerialNumber());
	}

	public String getCalibrationDataForModuleOnly() {
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_CALIBRATION_DATA_FOR_MODULE_ONLY,
			moduleIdentifier.getManufacturer(), 
			moduleIdentifier.getTypeNumber(), 
			moduleIdentifier.getSerialNumber());

		if (resultSet.length == 1){
			return (String) resultSet[0].get("properties");
		}
		System.out.println("Unable to find calibration entry for only this module (there might be a calibration entry shared with another module)");
		//throw KnowledgeDatabaseException();
		return null;
	}

	public String getCalibrationDataForModuleAndChilds() {
		System.out.println("getCalibrationDataForModuleAndChilds a1");
		Vector<ModuleIdentifier> childs = getChildModulesIdentifiers();
		// Is dit nog nodig? LARS System.out.println("getCalibrationDataForModuleAndChilds a2, vector size = " + childs.size());
		String returnValue = getCalibrationDataForModuleAndOtherModules(childs);
		System.out.println(returnValue);
		return returnValue;
	}

	private Vector<ModuleIdentifier> getChildModulesIdentifiers() {
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
					GET_CHILD_MODULES_INDENTIFIERS,
					moduleIdentifier.getManufacturer(), 
					moduleIdentifier.getTypeNumber(), 
					moduleIdentifier.getSerialNumber(),
					moduleIdentifier.getManufacturer(), 
					moduleIdentifier.getTypeNumber(), 
					moduleIdentifier.getSerialNumber());

		Vector<ModuleIdentifier> childModules = null;
		if (resultSet.length != 0){
			// get all the childs
			//while(result->next()){
			//	rexos_datatypes::ModuleIdentifier identifier(
			//		result->getString("manufacturer"),
			//		result->getString("typeNumber"),
			///		result->getString("serialNumber")
			//	);
			//	childModules.push_back(identifier);
			//}
		}
		return childModules;
	}


	public String getCalibrationDataForModuleAndOtherModules(Vector<ModuleIdentifier> moduleIdentifiers) {
		System.out.println("getCalibrationDataForModuleAndOtherModules b1" );

		int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
		String query = "SELECT properties FROM ModuleCalibration WHERE id = ?;";
		System.out.println("getCalibrationDataForModuleAndOtherModules b2, SQL query = " + query);

		//System.out.println("getCalibrationDataForModuleAndOtherModules b3, SQL preparedStatement = ");

		Row[] resultSet = knowledgeDBClient.executeSelectQuery(query, calibrationId);


		if(resultSet.length != 1){
			System.out.println("Unable to find calibration entry");
			return null;
		}
		return (String) resultSet[0].get("properties");
	}

	private int getCalibrationGroupForModuleAndOtherModules(
			Vector<ModuleIdentifier> moduleIdentifiers) {
		// TODO Auto-generated method stub
		return 0;
	}

	private int getCalibrationGroupForModuleAndOtherModules(ModuleIdentifier moduleIdentifiers) {
		// TODO Auto-generated method stub
		return 0;
	}


}
