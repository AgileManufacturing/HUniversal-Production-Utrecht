package HAL;

import generic.Mast;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Vector;

import org.json.JSONObject;

import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.ModuleTypeIdentifier;
import HAL.exceptions.FactoryException;
import HAL.factories.ModuleFactory;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;
import HAL.listeners.ModuleListener;

/**
 * Abstract representation of a module in the HAL.
 * @author Bas Voskuijlen
 * @author Lars Veenendaal
 *
 */
@SuppressWarnings("unused")
public class Module implements ModuleListener {
	protected KnowledgeDBClient knowledgeDBClient;
	protected ModuleIdentifier moduleIdentifier;
	protected ModuleFactory moduleFactory;
	protected ModuleListener moduleListener;
	private Mast.State state;
	private Mast.Mode mode;
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

	private static final String GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_TEMP_TABLE = 
		"CREATE TEMPORARY TABLE otherModules( " +
		"manufacturer char(200) NOT NULL, " +
		"typeNumber char(200) NOT NULL, " +
		"serialNumber char(200) NOT NULL);";

	private static final String GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_STORE_THE_MODULES = 
		"INSERT INTO otherModules(manufacturer, typeNumber, serialNumber) VALUES ( ?. ?. ? );";
	private static final String GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_ACTUAL_QUERY = 
		"SELECT id " +
		"FROM ModuleCalibration " +
		"JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id " +
		"WHERE manufacturer = ? AND " +
		"typeNumber = ? AND " +
		"serialNumber = ? AND " +
		"( " +
		"	SELECT count(*) " +
		"	FROM ModuleCalibrationModuleSet AS inListGroup " +
		"	JOIN otherModules ON " +
		"		inListGroup.manufacturer = otherModules.manufacturer AND " +
		"		inListGroup.typeKnowledgeExceptionNumber = otherModules.typeNumber AND " +
		"		inListGroup.serialNumber = otherModules.serialNumber " +
		"	WHERE ModuleCalibrationModuleSet.ModuleCalibration = inListGroup.ModuleCalibration " +
		") = ? AND " +
		"( " +
		"	SELECT count(*) " +
		"	FROM ModuleCalibrationModuleSet AS listGroup " +
		"	WHERE ModuleCalibrationModuleSet.ModuleCalibration = listGroup.ModuleCalibration AND ( " +
		"		listGroup.manufacturer != ModuleCalibrationModuleSet.manufacturer OR " +
		"		listGroup.typeNumber != ModuleCalibrationModuleSet.typeNumber OR " +
		"		listGroup.serialNumber != ModuleCalibrationModuleSet.serialNumber " +
		"	) " +
		") = ?;";
	private static final String GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_DROP_TABLE = 
		"DROP TEMPORARY TABLE otherModules;";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_ONLY_UPDATE_MODULECALIBRATION =
		"UPDATE ModuleCalibration " + 
		"SET properties = ? " + 
		"WHERE id = ?;";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_INSERT_MODULE_CALIBRATION_MODULE_SET =
		"INSERT INTO ModuleCalibrationModuleSet (properties) " + 
		"VALUES (?);";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_INSERT_MODULE_CALIBRATION =
		"INSERT INTO ModuleCalibration (ModuleCalibration, manufacturer, typeNumber, serialNumber) " + 
		"VALUES (LAST_INSERT_ID(), ?, ?, ?);";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_UPDATE_MODULE_CALIBRATION =
		"UPDATE ModuleCalibration " +
		"SET properties = ? " + 
		"WHERE id = ?;";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_INSERT_MODULE_CALIBRATION =
		"INSERT INTO ModuleCalibration (properties) " +
		"VALUES (?);";
	private static final String SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_INSERT_MODULE_CALIBRATION_MODULE_SET =
		"INSERT INTO ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) " + 
		"VALUES (LAST_INSERT_ID(), ?, ?, ?);";
	private static final String SET_MODULE_PROPERTIES = 
		"UPDATE Module " +
		"SET moduleProperties = ? " +
		"WHERE manufacturer = ? AND " +
		"typeNumber = ? AND " +
		"serialNumber = ?;";

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
																moduleIdentifier.manufacturer, 
																moduleIdentifier.typeNumber, 
																moduleIdentifier.serialNumber);
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
																moduleIdentifier.manufacturer,
																moduleIdentifier.typeNumber,
																moduleIdentifier.serialNumber,
																moduleIdentifier.manufacturer,
																moduleIdentifier.typeNumber,
																moduleIdentifier.serialNumber);
		
		if (resultSet.length == 1){
			ModuleIdentifier moduleIdentifier = new ModuleIdentifier(
					resultSet[0].get("manufacturer").toString(),
					resultSet[0].get("typeNumber").toString(),
					resultSet[0].get("serialNumber").toString());
			return this.moduleFactory.getItemForIdentifier(moduleIdentifier);
		}
		else return null;
	}

	/**
	 * This method will return the properties of this specific module
	 * @return
	 */
	public JSONObject getConfigurationProperties() {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * This method will return the properties of this specific module
	 * @return
	 */
	public JSONObject getProperties() {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * This method will forward the changed MAST module state to the {@link ModuleListener}
	 * Do not call this method!
	 * @author Auke de Witte
	 */
	@Override
	public void onModuleStateChanged(ModuleIdentifier module, Mast.State state){
		this.state = state;
		moduleListener.onModuleStateChanged(module, state);
	}
	/**
	 * This method will forward the changed MAST module mode to the {@link ModuleListener}
	 * Do not call this method!
	 */
	@Override
	public void onModuleModeChanged(ModuleIdentifier module, Mast.Mode mode){
		this.mode = mode;
		moduleListener.onModuleModeChanged(module, mode);
	}
	public Mast.State getModuleState(){
		return this.state;
	}
	
	public Mast.Mode getModuleMode(){
		return this.mode;
	}

	public int getMountPointX(){
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_MOUNT_POINT_X, 
			moduleIdentifier.manufacturer, 
			moduleIdentifier.typeNumber, 
			moduleIdentifier.serialNumber);

		if (resultSet.length == 1){
			return (int) resultSet[0].get("mountPointX");
		}
		return 0;
	} 

	public int getMountPointY(){
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_MOUNT_POINT_Y, 
			moduleIdentifier.manufacturer, 
			moduleIdentifier.typeNumber, 
			moduleIdentifier.serialNumber);

		if (resultSet.length == 1){
			return (int) resultSet[0].get("mountPointY");
		}
		return 0;
	}

	public String getCalibrationDataForModuleOnly() {
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
			GET_CALIBRATION_DATA_FOR_MODULE_ONLY,
			moduleIdentifier.manufacturer, 
			moduleIdentifier.typeNumber, 
			moduleIdentifier.serialNumber);

		if (resultSet.length == 1){
			return (String) resultSet[0].get("properties");
		}
		System.out.println("Unable to find calibration entry for only this module (there might be a calibration entry shared with another module)");
		//throw KnowledgeDatabaseException();
		return null;
	}

	public String getCalibrationDataForModuleAndChilds() {
		System.out.println("getCalibrationDataForModuleAndChilds a1");
		ArrayList<ModuleIdentifier> childs = getChildModulesIdentifiers();
		String returnValue = getCalibrationDataForModuleAndOtherModules(childs);
		System.out.println(returnValue);
		return returnValue;
	}

	private ArrayList<ModuleIdentifier> getChildModulesIdentifiers() {
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	
					GET_CHILD_MODULES_INDENTIFIERS,
					moduleIdentifier.manufacturer, 
					moduleIdentifier.typeNumber, 
					moduleIdentifier.serialNumber,
					moduleIdentifier.manufacturer, 
					moduleIdentifier.typeNumber, 
					moduleIdentifier.serialNumber);

		ArrayList<ModuleIdentifier> childModules = new ArrayList<ModuleIdentifier>();
		if (resultSet.length != 0){
			// get all the childs
			int i = 0;
			while(resultSet.length >= i){
				ModuleIdentifier indentifier = new ModuleIdentifier(
					resultSet[i].get("Manufacturer").toString(),
					resultSet[i].get("typeNumber").toString(),
					resultSet[i].get("serialNumber").toString()
				);
				childModules.add(indentifier);
				//childModules.push_back(indentifier);
				i++;
			}
		}
		return childModules;
	}


	public String getCalibrationDataForModuleAndOtherModules(ArrayList<ModuleIdentifier> moduleIdentifiers) {

		int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
		String query = "SELECT properties FROM ModuleCalibration WHERE id = ?;";
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(query, calibrationId);

		if(resultSet.length != 1){
			System.out.println("Unable to find calibration entry");
			return null;
		}
		return (String) resultSet[0].get("properties");
	}

	private int getCalibrationGroupForModuleAndOtherModules(ArrayList<ModuleIdentifier> moduleIdentifiers) {

		// create a temp table for storing the modules
		knowledgeDBClient.executeSelectQuery(GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_TEMP_TABLE);

		// Store the modules
		Iterator<ModuleIdentifier> itr = moduleIdentifiers.iterator();
		while(itr.hasNext()){
			knowledgeDBClient.executeSelectQuery(GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_STORE_THE_MODULES,
				((ModuleTypeIdentifier) itr).manufacturer,
				((ModuleTypeIdentifier) itr).typeNumber,
				((ModuleIdentifier) itr).serialNumber);
			itr.next();
		}

		// preform the actual query
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_ACTUAL_QUERY,
			moduleIdentifier.manufacturer,
			moduleIdentifier.typeNumber,
			moduleIdentifier.serialNumber,
			moduleIdentifiers.size(),
			moduleIdentifiers.size());

		if (resultSet.length != 0){
			System.out.println("result...");
			// delete the temp table for storing the modules
			knowledgeDBClient.executeSelectQuery(GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_DROP_TABLE);
			throw new KnowledgeException("Unable to find calibration entry for only this module and other modules");
		}
		knowledgeDBClient.executeSelectQuery(GET_CALIBRATION_GROUP_FOR_MODULE_AND_OTHER_MODULES_DROP_TABLE);

		return (int)resultSet[0].get("id");
	}

	// FIX DEZE!

	protected void setCalibrationDataForModuleOnly(String properties){
		
		ArrayList<ModuleIdentifier> emptyList = new ArrayList<ModuleIdentifier>();
		try{
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(emptyList);

			knowledgeDBClient.executeUpdateQuery(
			SET_CALIBRATION_DATA_FOR_MODULE_ONLY_UPDATE_MODULECALIBRATION,
			properties,
			calibrationId);

		}catch(KnowledgeException ex){

			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_INSERT_MODULE_CALIBRATION_MODULE_SET,
				properties); 

			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_INSERT_MODULE_CALIBRATION,
				moduleIdentifier.manufacturer,
				moduleIdentifier.typeNumber,
				moduleIdentifier.serialNumber);

		}

	}

	protected	void setCalibrationDataForModuleAndChilds(String properties){
		ArrayList<ModuleIdentifier> childs = getChildModulesIdentifiers();
		setCalibrationDataForModuleAndOtherModules(childs, properties);
	}

	protected void setCalibrationDataForModuleAndOtherModules(ArrayList<ModuleIdentifier> moduleIdentifiers, String properties){
		try{
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
			
			// update existing entry
			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_UPDATE_MODULE_CALIBRATION,
				properties,
				calibrationId);

		} catch (KnowledgeException ex) {
			// create a new entry
			 
			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_INSERT_MODULE_CALIBRATION,
				properties); 
			
			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_INSERT_MODULE_CALIBRATION_MODULE_SET,
				moduleIdentifier.manufacturer,
				moduleIdentifier.typeNumber,
				moduleIdentifier.serialNumber);
			
			//for(int i = 0; i < moduleIdentifiers.size(); i++){
			Iterator<ModuleIdentifier> itr = moduleIdentifiers.iterator();
			while(itr.hasNext()){
			knowledgeDBClient.executeSelectQuery(
				SET_CALIBRATION_DATA_FOR_MODULE_AND_OTHER_MODULE_INSERT_MODULE_CALIBRATION_MODULE_SET,
				((ModuleTypeIdentifier) itr).manufacturer,
				((ModuleTypeIdentifier) itr).typeNumber,
				((ModuleIdentifier) itr).serialNumber);
				itr.next();
			}
		}
	}

	protected	void setModuleProperties(String jsonProperties){
		knowledgeDBClient.executeUpdateQuery(
			SET_MODULE_PROPERTIES,
			jsonProperties,
			moduleIdentifier.manufacturer,
			moduleIdentifier.typeNumber,
			moduleIdentifier.serialNumber);
	}

	protected	void setMountPointX(int mountPointX){
		knowledgeDBClient.executeSelectQuery(	
			SET_MOUNT_POINT_X,
			moduleIdentifier.manufacturer, 
			moduleIdentifier.typeNumber,
			moduleIdentifier.serialNumber);
	}

	protected	void setMountPointY(int mountPointY){
		knowledgeDBClient.executeSelectQuery(	
			SET_MOUNT_POINT_Y,
			moduleIdentifier.manufacturer, 
			moduleIdentifier.typeNumber,
			moduleIdentifier.serialNumber);
	}

}