/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ReconfigFactory
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	This class is responsible for all modification of the KnowledgeDB regarding Modules and Capabilities
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-06-06
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	 Niek Arends
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2014, HU University of Applied Sciences Utrecht. 
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package HAL.factories;

import generic.Service;

import java.sql.SQLException;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.HashMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.JavaSoftware;
import HAL.Module;
import HAL.ModuleIdentifier;
import HAL.Mutation;
import HAL.RosSoftware;
import HAL.exceptions.FactoryException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

public class ReconfigFactory extends Factory {
	//Modules
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
	 * SQL query for adding a module which is connected to another module. The
	 * space required is the nested set tree is NOT inserted. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber, parentModuleManufacturer,
	 * parentModuleTypeNumber, parentModuleSerialNumber The module is added to
	 * the left of all the children of the parent module.
	 */
	// TODO store the input params so they don't have to be specified twice
	private static final String addModuleAttachedToModule = "INSERT INTO Module \n"
			+ "(manufacturer, typeNumber, serialNumber, moduleProperties, equiplet, attachedToLeft, attachedToRight) \n"
			+ "VALUES (?, ?, ?, ?, ?, ( \n"
			+ "	SELECT attachedToLeft + 1 \n"
			+ "	FROM (SELECT * FROM Module) AS tbl1 \n"
			+ "	WHERE manufacturer = ? AND \n"
			+ "		typeNumber = ? AND \n"
			+ "		serialNumber = ? \n"
			+ "), ( \n"
			+ "	SELECT attachedToLeft + 2 \n"
			+ "	FROM (SELECT * FROM Module) AS tbl2 \n"
			+ "	WHERE manufacturer = ? AND \n"
			+ "		typeNumber = ? AND \n"
			+ "		serialNumber = ? \n" + "));";

	/**
	 * SQL query for adding a moduleType. Input: moduleTypeManufacturer,
	 * moduleTypeTypeNumber, halSoftwareId, rosSoftwareId Ignored if a record
	 * with the same primary key already exists.
	 */
	private static final String addModuleType = "INSERT IGNORE INTO ModuleType \n"
			+ "(manufacturer, typeNumber, moduleTypeProperties, halSoftware, rosSoftware) \n"
			+ "VALUES (?, ?, ?, ?, ?);";

	/**
	 * SQL query for inserting the left space in the nested set tree for a
	 * module which is connected to another module. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber
	 */
	private static final String insertSpaceInNestedTreeForModuleLeft = "UPDATE Module \n"
			+ "SET attachedToLeft = attachedToLeft + 2 \n"
			+ "WHERE attachedToLeft >= ( \n"
			+ "	SELECT attachedToRight \n"
			+ "	FROM (SELECT * FROM Module) AS tbl1 \n"
			+ "	WHERE manufacturer = ? AND \n"
			+ "		typeNumber = ? AND \n"
			+ "		serialNumber = ? \n" + ");";
	/**
	 * SQL query for inserting the right space in the nested set tree for a
	 * module which is connected to another module. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber
	 */
	private static final String insertSpaceInNestedTreeForModuleRight = "UPDATE Module \n"
			+ "SET attachedToRight = attachedToRight + 2 \n"
			+ "WHERE attachedToRight >= ( \n"
			+ "	SELECT attachedToRight \n"
			+ "	FROM (SELECT * FROM Module) AS tbl1 \n"
			+ "	WHERE manufacturer = ? AND \n"
			+ "		typeNumber = ? AND \n"
			+ "		serialNumber = ? \n" + ");";

	/**
	 * SQL query for removing ModuleCalibrationData associated with a module
	 * (which is identified using a {@link ModuleIdentifier}). This effectively
	 * removes obsolete ModuleCalibrationData. Input: moduleManufacturer,
	 * moduleTypeNumber, moduleSerialNumber ModuleCalibrationData is associated
	 * when at least one of the ModuleIdentifiers in the module set matches the
	 * ModuleIdentifier of this module. ModuleCalibrationData is considered to
	 * be obsolete when at least one of the ModuleIdentifiers in the no longer
	 * matches the ModuleIdentifier of the modules attached to the equiplet.
	 */
	private static final String removeAllCalibrationDataForModule = "DELETE FROM ModuleCalibration \n"
			+ "WHERE id IN( \n"
			+ "	SELECT ModuleCalibration \n"
			+ "	FROM ModuleCalibrationModuleSet \n"
			+ "	WHERE manufacturer = ? AND \n"
			+ "		typeNumber = ? AND \n"
			+ "		serialNumber = ? \n" + "); \n";

	/**
	 * SQL query for removing a module. Input: moduleManufacturer,
	 * moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeModule = "DELETE FROM Module \n"
			+ "WHERE manufacturer = ? AND \n" + "	typeNumber = ? AND \n"
			+ "	serialNumber = ?;";
	
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
	//Capabilities
	/**
	 * SQL query for adding a capabilityType.
	 * Input: capabilityTypeName, halSoftwareId
	 */
	private static final String addCapabilityType =
			"INSERT IGNORE INTO CapabilityType \n" + 
			"(name, halSoftware) \n" + 
			"VALUES(?, ?);";
	
	/**
	 * SQL query for adding a serviceType.
	 * Input: serviceTypeName
	 */
	private static final String addServiceType =
			"INSERT IGNORE INTO ServiceType \n" + 
			"(name) \n" + 
			"VALUES(?);";
	
	/**
	 * SQL query for adding a required mutation to a capabilityType.
	 * Input: treeNumber, capabilityTypeName, mutation
	 */
	private static final String addRequiredMutationForCapabilityType =
			"INSERT IGNORE INTO CapabilityTypeRequiredMutation \n" + 
			"(treeNumber, capabilityType, mutation) \n" + 
			"VALUES(?, ?, ?);";
	
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
	 * SQL query for selecting the supported serviceTypes for an equiplet.
	 * Input: equipletName
	 * A serviceType is supported when at least one capabilityType is supported. 
	 * A capabilityType is supported when all the function module trees could be matched with the corresponding physical module trees.
	 */
	private static final String getSupportedServiceTypes = 
			"SELECT serviceType \n" + 
			"FROM ServiceType_CapabilityType \n" + 
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM CapabilityTypeRequiredMutation \n" +
			"	WHERE ServiceType_CapabilityType.capabilityType = CapabilityTypeRequiredMutation.capabilityType AND \n" +
			"	treeNumber NOT IN( \n" +
			"		SELECT treeNumber \n" +
			"		FROM CapabilityTypeRequiredMutation AS currentRequiredMutation \n" +
			"		JOIN Module AS currentModule \n" +
			"		WHERE ServiceType_CapabilityType.capabilityType = currentRequiredMutation.capabilityType AND \n" +
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
	 * SQL query for selecting required mutations for a capabilityType.
	 * Input: capabilityTypeName
	 */
	public static final String getRequiredMutationsForCapabilityType =
			"SELECT mutation, treeNumber \n" + 
			"FROM CapabilityTypeRequiredMutation \n" + 
			"WHERE capabilityType = ?;";
	
	/**
	 * SQL query for selecting all the associated capabilityTypes to a ModuleIdentifier.
	 * Input: ModuleIdentifierManufacturer, ModuleIdentifierTypeNumber
	 * A capabilityTypes is considered associated when at least one required mutation matches with a supported mutation of this module type (which is identified with by {@link ModuleIdentifier}).
	 */
	public static final String getAllAssociatedCapabilityTypesForModuleIdentifier = 
			"SELECT DISTINCT capabilityType \n" + 
			"FROM CapabilityTypeRequiredMutation \n" + 
			"WHERE mutation IN( \n" + 
			"	SELECT mutation \n" + 
			"	FROM SupportedMutation \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");";
	
	/**
	 * SQL query for selecting all the associated ModuleCalibrationData for a module (which is identified using a {@link ModuleIdentifier}).
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 * ModuleCalibrationData is associated when at least one of the ModuleIdentifiers in the module set matches the ModuleIdentifier of this module.
	 */
	public static final String getAllModuleCalibrationDataForModule =
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
	 * SQL query for selecting all the data of moduleType.
	 * Input: moduleTypeManufacturer, moduleTypeTypeNumber
	 */
	public static final String getModuleType =
			"SELECT * \n" +
			"FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ?;"; 

	/**
	 * SQL query for selecting all the {@link ModuleIdentifier} in the moduleSet of the ModuleCalibrationData.
	 * Input: ModuleCalibrationId
	 */
	public static final String getModuleSetForModuleCalibrationData =
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM ModuleCalibrationModuleSet \n" + 
			"WHERE ModuleCalibration = ?;";	
	
	/**
	 * SQL query for selecting all the data of a module 
	 * Input: moduleManufacturer, moduleTypeNumber
	 */
	public static final String getModule =
			"SELECT * \n" +
			"FROM Module \n" +
			"WHERE manufacturer = ? AND \n" + 
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;"; 
	
	/**
	 * This method will serialize all the capabilityTypes associated with the moduleType (which is identified by the {@link ModuleIdentifier}).
	 * This method will also remove all the capabilityTypes which have become obsolete after removing the module type. 
	 * A capabilityType is considered to be obsolete if none of the required mutations matches a supported mutation.
	 * @param moduleIdentifier
	 * @return The serialized associated capabilities.
	 */
	public JSONArray removeCapabilities(CapabilityFactory cf, ModuleIdentifier moduleIdentifier) {
		ArrayList<String> capabilityNames = new ArrayList<String>();
		try{
			try{
				Row[] rows = knowledgeDBClient.executeSelectQuery(getAllAssociatedCapabilityTypesForModuleIdentifier, 
						moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
				for (Row row : rows) {
					capabilityNames.add((String) row.get("capabilityType"));
				}
				
				JSONArray capabilities = new JSONArray();
				for (String capabilityName : capabilityNames) {
					JSONObject capability = new JSONObject();
					capability.put("name", capabilityName);
					
					JavaSoftware javaSoftware = JavaSoftware.getJavaSoftwareForCapabilityName(capabilityName);
					capability.put("halSoftware", javaSoftware.serialize());
					
					capability.put("requiredMutationsTrees", serializeRequiredMutations(capabilityName));
					
					//TODO actually remove the capability
					JSONArray services = new JSONArray();
					Row[] serviceRows = knowledgeDBClient.executeSelectQuery(cf.getServiceTypesForCapabilityType, capabilityName);
					for (Row serviceRow : serviceRows) {
						services.put(serviceRow.get("serviceType"));
					}
					capability.put("services", services);
					
					capabilities.put(capability);
				}
				return capabilities;
			} catch(Exception ex) {
				Logger.log(LogSection.HAL_CAPABILITY_FACTORY, LogLevel.WARNING, "Error occured while removing capability ", ex);
				knowledgeDBClient.getConnection().rollback();
				knowledgeDBClient.getConnection().setAutoCommit(true);
				return null;
			}
		} catch (SQLException ex) {
			return null;
		}
	}

	private HardwareAbstractionLayer hal;

	public ReconfigFactory(HardwareAbstractionLayer hal) {
		super(new KnowledgeDBClient());
		this.hal = hal;
	}
	
	/**
	 * SQL query for adding a relation between a serviceType and a capabilityType.
	 * Input: serviceTypeName, capabilityTypeName
	 */
	private static final String addServiceType_CapabilityType = 
			"INSERT IGNORE INTO ServiceType_CapabilityType \n" + 
					"(serviceType,capabilityType) \n" + 
					"VALUES(?, ?);";

	/**
	 * This methods attempts to insert a module in the database using the data
	 * provided in the JSONObjects.
	 * 
	 * @param staticSettings
	 *            contains all the static information of the module (software,
	 *            properties, calibrationData, etc).
	 * @param dynamicSettings
	 *            contains all the dynamic information of the module (mount
	 *            position, attached to other modules, orientation, etc).
	 * @return true if insertion of the module is successful, false otherwise.
	 */
	
//MODULE
	public boolean insertModule(JSONObject staticSettings,
			JSONObject dynamicSettings) {
		try {
			try {
				knowledgeDBClient.getConnection().setAutoCommit(false);
				ModuleIdentifier moduleIdentifier = new ModuleIdentifier(
						staticSettings.getString("manufacturer"),
						staticSettings.getString("typeNumber"),
						staticSettings.getString("serialNumber"));

				if (isModuleTypeKnown(moduleIdentifier)) {
					updateModuleType(moduleIdentifier,
							staticSettings.getJSONObject("type"));
				} else {
					insertModuleType(moduleIdentifier,
							staticSettings.getJSONObject("type"));
				}

				String properties = staticSettings.getString("properties");

				if (dynamicSettings.isNull("attachedTo")) {
					// we are not attached to another module
					Integer mountPointX = dynamicSettings.getInt("mountPointX");
					Integer mountPointY = dynamicSettings.getInt("mountPointY");
					knowledgeDBClient.executeUpdateQuery(addTopModule,
							moduleIdentifier.getManufacturer(),
							moduleIdentifier.getTypeNumber(),
							moduleIdentifier.getSerialNumber(), properties,
							hal.getEquipletName(), mountPointX, mountPointY);
				} else if (dynamicSettings.isNull("mountPointX")
						|| dynamicSettings.isNull("mountPointY")) {
					// this module is attached to another module
					JSONObject parentModuleJson = dynamicSettings
							.getJSONObject("attachedTo");
					ModuleIdentifier parentModuleIdentifier = new ModuleIdentifier(
							parentModuleJson.getString("manufacturer"),
							parentModuleJson.getString("typeNumber"),
							parentModuleJson.getString("serialNumber"));

					insertSpace(parentModuleIdentifier);
					knowledgeDBClient.executeUpdateQuery(
							addModuleAttachedToModule,
							moduleIdentifier.getManufacturer(),
							moduleIdentifier.getTypeNumber(),
							moduleIdentifier.getSerialNumber(), properties,
							hal.getEquipletName(),
							parentModuleIdentifier.getManufacturer(),
							parentModuleIdentifier.getTypeNumber(),
							parentModuleIdentifier.getSerialNumber(),
							parentModuleIdentifier.getManufacturer(),
							parentModuleIdentifier.getTypeNumber(),
							parentModuleIdentifier.getSerialNumber());
				} else {
					throw new FactoryException(
							"Module both attached to the mountplate and a module");
				}

				// calibration
				JSONArray calibrationEntries = staticSettings
						.getJSONArray("calibrationData");
				deserializeModuleCalibrationData(calibrationEntries);

			} catch (Exception ex) {
				Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.ERROR,
						"Error occured while inserting module ", ex);
				knowledgeDBClient.getConnection().rollback();
				knowledgeDBClient.getConnection().setAutoCommit(true);
				return false;
			}
			knowledgeDBClient.getConnection().commit();
			knowledgeDBClient.getConnection().setAutoCommit(true);
			return true;
		} catch (SQLException ex) {
			return false;
		}
	}

	/**
	 * This methods attempts to update a module in the database using the data
	 * provided in the JSONObjects.
	 * 
	 * @param staticSettings
	 *            contains all the static information of the module (software,
	 *            properties, calibrationData, etc).
	 * @return true if insertion of the module is successful, false otherwise.
	 */
	public boolean updateModule(JSONObject staticSettings,
			JSONObject dynamicSettings) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * This method updates a moduleType in the knowledge database. It will
	 * update the software of the moduleType if the buildNumber of the provided
	 * software is higher than the buildNumber of the currently stored software.
	 * 
	 * @param moduleIdentifier
	 * @param type
	 * @throws JSONException
	 */
	private void updateModuleType(ModuleIdentifier moduleIdentifier,
			JSONObject type) throws JSONException {
		JSONObject halSoftwareObject = type.getJSONObject("halSoftware");

		JavaSoftware javaSoftware = JavaSoftware
				.getJavaSoftwareForModuleIdentifier(moduleIdentifier,
						knowledgeDBClient);
		int currentJavaSoftwareBuildNumber = javaSoftware.getBuildNumber();
		int newJavaSoftwareBuildNumber = JavaSoftware
				.getBuildNumber(halSoftwareObject);
		if (newJavaSoftwareBuildNumber > currentJavaSoftwareBuildNumber) {
			// update the halSoftware
			Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION,
					"Updating HAL software for module " + moduleIdentifier);
			javaSoftware.updateJavaSoftware(halSoftwareObject);
		}

		JSONObject rosSoftwareObject = type.getJSONObject("rosSoftware");
		RosSoftware rosSoftware = RosSoftware
				.getRosSoftwareForModuleIdentifier(moduleIdentifier,
						knowledgeDBClient);
		int currentRosSoftwareBuildNumber = rosSoftware.getBuildNumber();
		int newRosSoftwareBuildNumber = RosSoftware
				.getBuildNumber(rosSoftwareObject);
		if (newRosSoftwareBuildNumber > currentRosSoftwareBuildNumber) {
			// update the rosSoftware
			Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION,
					"Updating ROS software for module " + moduleIdentifier);
			rosSoftware.updateRosSoftware(rosSoftwareObject);
		}
	}

	/**
	 * This method will serialize all the moduleCalibrationData associated with
	 * the module identified by the {@link ModuleIdentifier}. This method will
	 * NOT remove the serialized moduleCalibrationData.
	 * 
	 * @param moduleIdentifier
	 * @return
	 * @throws JSONException
	 */
	private JSONArray serializeModuleCalibrationData(ModuleIdentifier moduleIdentifier) throws JSONException {
		JSONArray calibrationEntries = new JSONArray();
		Row[] calibrationDataRows = knowledgeDBClient.executeSelectQuery(
				getAllModuleCalibrationDataForModule,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL,
				"getAllModuleCalibrationDataForModule", calibrationDataRows);
		for (Row calibrationDataRow : calibrationDataRows) {
			Integer moduleCalibrationId = (Integer) calibrationDataRow
					.get("id");
			String dateTime = ((Timestamp) calibrationDataRow.get("date"))
					.toString();
			String properties = (String) calibrationDataRow.get("properties");

			JSONObject calibrationDataEntry = new JSONObject();
			calibrationDataEntry.put("date", dateTime);
			calibrationDataEntry.put("data", properties);

			// fetch the moduleSet for the calibration data
			JSONArray moduleEntries = new JSONArray();
			Row[] moduleSetrows = knowledgeDBClient.executeSelectQuery(
					getModuleSetForModuleCalibrationData,
					moduleCalibrationId);
			logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL,
					"getModuleSetForModuleCalibrationData", calibrationDataRows);
			for (Row moduleSetrow : moduleSetrows) {
				String manufacturer = (String) moduleSetrow.get("manufacturer");
				String typeNumber = (String) moduleSetrow.get("typeNumber");
				String serialNumber = (String) moduleSetrow.get("serialNumber");

				JSONObject moduleEntry = new JSONObject();
				moduleEntry.put("manufacturer", manufacturer);
				moduleEntry.put("typeNumber", typeNumber);
				moduleEntry.put("serialNumber", serialNumber);

				moduleEntries.put(moduleEntry);
			}
			calibrationDataEntry.put("moduleSet", moduleEntries);

			calibrationEntries.put(calibrationDataEntry);
		}
		return calibrationEntries;
	}

	/**
	 * This method inserts a moduleType in the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @param type
	 * @return
	 * @throws KnowledgeException
	 */
	private boolean insertModuleType(ModuleIdentifier moduleIdentifier,
			JSONObject type) throws KnowledgeException {
		try {
			JSONObject halSoftwareObject = type.getJSONObject("halSoftware");
			JavaSoftware halSoftware = JavaSoftware.insertJavaSoftware(
					halSoftwareObject, knowledgeDBClient);
			int halSoftwareId = halSoftware.getId();

			// not every module has rosSoftware
			Integer rosSoftwareId = null;
			if (type.isNull("rosSoftware") == false) {
				JSONObject rosSoftwareObject = type
						.getJSONObject("rosSoftware");
				RosSoftware rosSoftware = RosSoftware.insertRosSoftware(
						rosSoftwareObject, knowledgeDBClient);
				rosSoftwareId = rosSoftware.getId();
			}

			String properties = type.getString("properties");
			knowledgeDBClient.executeUpdateQuery(addModuleType,
					moduleIdentifier.getManufacturer(),
					moduleIdentifier.getTypeNumber(), properties,
					halSoftwareId, rosSoftwareId);

			JSONArray supportedMutationEntries = type
					.getJSONArray("supportedMutations");
			Mutation.insertSupportedMutations(moduleIdentifier,
					supportedMutationEntries, knowledgeDBClient);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.ERROR,
					"Unable to insert module due to illegally formatted JSON",
					ex);
		}
		return true;
	}

	/**
	 * This method will deserialize all the moduleCalibration data provided and
	 * store it in the knowledge database.
	 * 
	 * @param moduleCalibrationEntries
	 * @throws JSONException
	 */
	private void deserializeModuleCalibrationData(
			JSONArray moduleCalibrationEntries) throws JSONException {
		for (int i = 0; i < moduleCalibrationEntries.length(); i++) {
			JSONObject moduleCalibrationEntry = moduleCalibrationEntries
					.getJSONObject(i);
			String dateTime = moduleCalibrationEntry.getString("date");
			String properties = moduleCalibrationEntry.getString("data");
			int calibrationDataId = knowledgeDBClient.executeUpdateQuery(
					addModuleCalibrationData, dateTime, properties);

			JSONArray moduleSetEntries = moduleCalibrationEntry
					.getJSONArray("moduleSet");
			for (int j = 0; j < moduleSetEntries.length(); j++) {
				JSONObject moduleSetEntry = moduleSetEntries.getJSONObject(j);

				String manufacturer = moduleSetEntry.getString("manufacturer");
				String typeNumber = moduleSetEntry.getString("typeNumber");
				String serialNumber = moduleSetEntry.getString("serialNumber");
				knowledgeDBClient.executeUpdateQuery(
						addModuleToCalibrationData, calibrationDataId,
						manufacturer, typeNumber, serialNumber);
			}
		}
	}

	/**
	 * This method inserts space in the nested set tree for a module to be
	 * attached to the parentModule.
	 * 
	 * @param parentModuleIdentifier
	 * @throws KnowledgeException
	 */
	private void insertSpace(ModuleIdentifier parentModuleIdentifier)
			throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(
				insertSpaceInNestedTreeForModuleLeft,
				parentModuleIdentifier.getManufacturer(),
				parentModuleIdentifier.getTypeNumber(),
				parentModuleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(
				insertSpaceInNestedTreeForModuleRight,
				parentModuleIdentifier.getManufacturer(),
				parentModuleIdentifier.getTypeNumber(),
				parentModuleIdentifier.getSerialNumber());
	}

	/**
	 * This method removes a module from the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @return the static information of the module.
	 * @throws JSONException
	 */
	public JSONObject removeModule(ModuleFactory mf,
			ModuleIdentifier moduleIdentifier) throws JSONException {
		JSONObject output = new JSONObject();
		output.put("manufacturer", moduleIdentifier.getManufacturer());
		output.put("typeNumber", moduleIdentifier.getTypeNumber());
		output.put("serialNumber", moduleIdentifier.getSerialNumber());

		JSONObject type = new JSONObject();
		Module module = mf.getModuleByIdentifier(moduleIdentifier);
		String moduleProperties = module.getProperties();
		type.put("properties", moduleProperties);

		// fetch halSoftware
		JavaSoftware halSoftware = JavaSoftware
				.getJavaSoftwareForModuleIdentifier(moduleIdentifier);
		type.put("halSoftware", halSoftware.serialize());
		// fetch rosSoftware
		RosSoftware rosSoftware = RosSoftware
				.getRosSoftwareForModuleIdentifier(moduleIdentifier);
		type.put("rosSoftware", rosSoftware.serialize());

		type.put("supportedMutations", Mutation.serializeAllSupportedMutations(
				moduleIdentifier, knowledgeDBClient));
		Mutation.removeSupportedMutations(moduleIdentifier, knowledgeDBClient);

		output.put("calibrationData",
				serializeModuleCalibrationData(moduleIdentifier));
		knowledgeDBClient.executeUpdateQuery(removeAllCalibrationDataForModule,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());

		Row[] moduleTypeRows = knowledgeDBClient.executeSelectQuery(
				getModuleType, moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getModuleType",
				moduleTypeRows);
		type.put("properties",
				(String) moduleTypeRows[0].get("moduleTypeProperties"));

		Row[] moduleRows = knowledgeDBClient.executeSelectQuery(getModule,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getModule", moduleRows);
		output.put("properties", (String) moduleRows[0].get("moduleProperties"));

		removeSpace(moduleIdentifier);
		knowledgeDBClient.executeUpdateQuery(removeModule,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(removeModuleTypesWithNoModules);

		output.put("type", type);
		return output;
	}

	/**
	 * This method removes space in the nested set tree for a module.
	 * 
	 * @param moduleIdentifier
	 *            is the identifier of the module to be removed.
	 * @throws KnowledgeException
	 */
	private void removeSpace(ModuleIdentifier moduleIdentifier)
			throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(
				removeSpaceInNestedTreeForModuleLeft,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());
		knowledgeDBClient.executeUpdateQuery(
				removeSpaceInNestedTreeForModuleRight,
				moduleIdentifier.getManufacturer(),
				moduleIdentifier.getTypeNumber(),
				moduleIdentifier.getSerialNumber());
	}
	
	//CAPABILITIES
	/**
	 * This method will insert a array of capabilityTypes into the knowledge database, using the data provided in the JSONArray.
	 * @param capabilityTypes
	 * @return true if successful, false otherwise
	 */
	public boolean insertCapabilityTypes(JSONArray capabilityTypes) {
		try{
			try{
				for (int i = 0; i < capabilityTypes.length(); i++) {
					JSONObject capabilityTypeEntry = capabilityTypes.getJSONObject(i);
					String name = capabilityTypeEntry.getString("name");
					
					JSONObject capabilitySoftware = capabilityTypeEntry.getJSONObject("halSoftware");
					JavaSoftware halSoftware = JavaSoftware.insertJavaSoftware(capabilitySoftware, knowledgeDBClient);
					int halSoftwareId = halSoftware.getId();
					
					knowledgeDBClient.executeUpdateQuery(addCapabilityType, name, halSoftwareId);
					//TODO update behavior for the required mutations
					JSONArray requiredMutationsTrees = capabilityTypeEntry.getJSONArray("requiredMutationsTrees");
					deserializeRequiredMutations(name, requiredMutationsTrees);
					
					JSONArray services = capabilityTypeEntry.getJSONArray("services");
					for (int j = 0; j < services.length(); j++) {
						String serviceName = services.getString(j);
						knowledgeDBClient.executeUpdateQuery(addServiceType, serviceName);
						knowledgeDBClient.executeUpdateQuery(addServiceType_CapabilityType, serviceName,name);
					}
				}
			} catch(Exception ex) {
				Logger.log(LogSection.HAL_CAPABILITY_FACTORY, LogLevel.WARNING, "Error occured while inserting capability ", ex);
				knowledgeDBClient.getConnection().rollback();
				knowledgeDBClient.getConnection().setAutoCommit(true);
				return false;
			}
		} catch (SQLException ex) {
			return false;
		}
		return true;
	}
	
	/**
	 * This method deserializes the required mutations and stores them in the knowledge database.
	 * @param capabilityTypeName
	 * @param requiredMutationTrees
	 * @throws JSONException 
	 */
	private void deserializeRequiredMutations(String capabilityTypeName, JSONArray requiredMutationTrees) throws JSONException {
		for (int i = 0; i < requiredMutationTrees.length(); i++) {
			JSONObject requiredMutationTree = requiredMutationTrees.getJSONObject(i);
			Integer requiredMutationTreeNumber = requiredMutationTree.getInt("treeNumber");
			JSONArray requiredMutations = requiredMutationTree.getJSONArray("mutations");
			for (int j = 0; j < requiredMutations.length(); j++) {
				String requiredMutation = requiredMutations.getString(j);
				knowledgeDBClient.executeUpdateQuery(addRequiredMutationForCapabilityType, 
						requiredMutationTreeNumber, capabilityTypeName, requiredMutation);
			}
		}
	}
	
	/**
	 * This method will serialize all the required mutations of a capabilityType from the knowledge database, but does NOT remove them.
	 * @param capabilityTypeName
	 * @return
	 * @throws JSONException 
	 */
	private JSONArray serializeRequiredMutations(String capabilityTypeName) throws JSONException {
		HashMap<Integer, JSONObject> requiredTreesMap = new HashMap<Integer, JSONObject>();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRequiredMutationsForCapabilityType, 
				capabilityTypeName);
		for (Row row : rows) {
			Integer treeNumber = (Integer) row.get("treeNumber");
			String mutation = (String) row.get("mutation");
			
			if(requiredTreesMap.containsKey(treeNumber) == false) {
				JSONObject tree = new JSONObject();
				tree.put("treeNumber", treeNumber);
				tree.put("mutations", new JSONArray());
				requiredTreesMap.put(treeNumber, tree);
			}
			
			requiredTreesMap.get(treeNumber).getJSONArray("mutations").put(mutation);
		}
		JSONArray requiredMutationTrees = new JSONArray();
		for (JSONObject entry : requiredTreesMap.values()) {
			requiredMutationTrees.put(entry);
		}
		return requiredMutationTrees;
	}
	
	/**
	 * This method will all return all the services supported by this equiplet.
	 * @return
	 */
	public ArrayList<Service> getAllSupportedServices() {
		ArrayList<Service> services = new ArrayList<Service>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedServiceTypes, hal.getEquipletName());
		for (Row row : rows) {
			String serviceName = (String) row.get("serviceType");
			services.add(new Service(serviceName));
		}
		return services;
	}
	
	/**
	 * This method determines if a moduleType (identified by the {@link ModuleIdentifier}) is known in the knowledge database.
	 * @param moduleIdentifier
	 * @return true if the moduleType is known in the knowledge database, false otherwise.
	 */
	public boolean isModuleTypeKnown(ModuleIdentifier moduleIdentifier) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		logSqlResult(LogSection.HAL_MODULE_FACTORY_SQL, "getModuleType", rows);
		if(rows.length == 1) {
			return true;
		} else {
			return false;
		}
	}
}
