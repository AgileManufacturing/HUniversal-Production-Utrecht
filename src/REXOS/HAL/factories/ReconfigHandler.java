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

import java.sql.SQLException;
import java.text.ParseException;
import java.util.ArrayList;

import org.json.JSONException;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.dataTypes.CalibrationEntry;
import HAL.dataTypes.DynamicSettings;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.ModuleType;
import HAL.dataTypes.StaticSettings;
import HAL.exceptions.FactoryException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

public class ReconfigHandler {
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
	private static final String addAttachedToModule = 
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
	 * SQL query for adding a module which is connected to the mountPlate.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber, moduleProperties, equiplet, mountPointX, mountPointY
	 * The module is added to the right of the nested set tree.
	 */
	private static final String updateModuleToTopModule = 
			"UPDATE Module \n" +
			"SET manufacturer = ? AND \n" +
			"typeNumber = ? AND \n" +
			"serialNumber = ? AND \n" +
			"moduleProperties = ? AND \n" +
			"typeNumber = ? AND \n" +
			"equiplet = ? AND \n" +
			"mountPointX = NULL AND \n" +
			"mountPointY = NULL AND \n" +
			"attachedToLeft = IFNULL( ( \n" +
			"		SELECT max(attachedToRight) + 1 \n" +
			"		FROM (SELECT * FROM Module) AS tbl1 \n" +
			"	), ( \n" +
			"		1 \n" +
			"	) )  AND \n" +
			"attachedToRight = IFNULL( ( \n" +
			"		SELECT max(attachedToRight) + 2 \n" +
			"		FROM (SELECT * FROM Module) AS tbl2 \n" +
			"	), ( \n" +
			"		2 \n" +
			"	) );";
	/**
	 * SQL query for adding a module which is connected to another module. The
	 * space required is the nested set tree is NOT inserted. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber, parentModuleManufacturer,
	 * parentModuleTypeNumber, parentModuleSerialNumber The module is added to
	 * the left of all the children of the parent module.
	 */
	// TODO store the input params so they don't have to be specified twice
	private static final String updateModuleToAttachedToModule = 
			"UPDATE Module \n" +
			"SET manufacturer = ? AND \n" +
			"typeNumber = ? AND \n" +
			"serialNumber = ? AND \n" +
			"moduleProperties = ? AND \n" +
			"typeNumber = ? AND \n" +
			"equiplet = ? AND \n" +
			"mountPointX = ? AND \n" +
			"mountPointY = ? AND \n" +
			"attachedToLeft = ( \n" +
			"	SELECT attachedToLeft + 1 \n" +
			"	FROM (SELECT * FROM Module) AS tbl1 \n" +
			"	WHERE manufacturer = ? AND \n" +
			"		typeNumber = ? AND \n" +
			"		serialNumber = ? \n" +
			") AND \n" +
			"attachedToRight = ( \n" +
			"	SELECT attachedToLeft + 2 \n" +
			"	FROM (SELECT * FROM Module) AS tbl2 \n" +
			"	WHERE manufacturer = ? AND \n" +
			"		typeNumber = ? AND \n" +
			"		serialNumber = ? \n" +
			");";
	/**
	 * SQL query for inserting the left space in the nested set tree for a
	 * module which is connected to another module. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber
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
	 * SQL query for inserting the right space in the nested set tree for a
	 * module which is connected to another module. Input:
	 * parentModuleManufacturer, parentModuleTypeNumber,
	 * parentModuleSerialNumber
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
	 * SQL query for removing a module. Input: moduleManufacturer,
	 * moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeModule = "" +
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

	private KnowledgeDBClient knowledgeDBClient;
	private HardwareAbstractionLayer hal;
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;

	public ReconfigHandler(HardwareAbstractionLayer hal, CapabilityFactory capabilityFactory, ModuleFactory moduleFactory) {
		this.hal = hal;
		this.capabilityFactory = capabilityFactory;
		this.moduleFactory = moduleFactory;
		knowledgeDBClient = new KnowledgeDBClient();
	}

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
	public boolean insertModule(StaticSettings staticSettings, DynamicSettings dynamicSettings) throws FactoryException {
		try {
			try {
				knowledgeDBClient.getConnection().setAutoCommit(false);
				
				staticSettings.moduleType.insertIntoDatabase(knowledgeDBClient);
				
				if (dynamicSettings.mountPointX != null && dynamicSettings.mountPointY != null) {
					// we are not attached to another module
					knowledgeDBClient.executeUpdateQuery(addTopModule, 
							staticSettings.moduleIdentifier.manufacturer, staticSettings.moduleIdentifier.typeNumber, staticSettings.moduleIdentifier.serialNumber, 
							staticSettings.moduleConfigurationProperties, hal.getEquipletName(), 
							dynamicSettings.mountPointX, dynamicSettings.mountPointY);
				} else if (dynamicSettings.attachedTo != null) {
					// this module is attached to another module
					ModuleIdentifier parentModuleIdentifier = dynamicSettings.attachedTo;
					insertSpace(parentModuleIdentifier);
					knowledgeDBClient.executeUpdateQuery(addAttachedToModule, 
							staticSettings.moduleIdentifier.manufacturer, staticSettings.moduleIdentifier.typeNumber, staticSettings.moduleIdentifier.serialNumber, 
							staticSettings.moduleConfigurationProperties, hal.getEquipletName(), 
							parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber, 
							parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber);
				} else {
					throw new FactoryException("Module both attached to the mountplate and a module or neither");
				}
				
				// calibration
				for (CalibrationEntry calibrationEntry : staticSettings.calibrationData) {
					calibrationEntry.insertIntoDatabase(knowledgeDBClient);
				}
				knowledgeDBClient.getConnection().commit();
				return true;
			} catch (Exception ex) {
				Logger.log(LogSection.HAL_RECONFIG, LogLevel.ERROR, "Error occured while inserting module ", ex);
				knowledgeDBClient.getConnection().rollback();
				return false;
			} finally {
				knowledgeDBClient.getConnection().setAutoCommit(true);
			}
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
	public boolean updateModule(StaticSettings staticSettings, DynamicSettings dynamicSettings) {
		try {
			try {
				knowledgeDBClient.getConnection().setAutoCommit(false);
				
				staticSettings.moduleType.insertIntoDatabase(knowledgeDBClient);
				
				// first remove the space, then reallocate it
				removeSpace(staticSettings.moduleIdentifier);
				
				if (dynamicSettings.mountPointX != null && dynamicSettings.mountPointY != null) {
					// we are not attached to another module
					knowledgeDBClient.executeUpdateQuery(updateModuleToTopModule, 
							staticSettings.moduleIdentifier.manufacturer, staticSettings.moduleIdentifier.typeNumber, staticSettings.moduleIdentifier.serialNumber, 
							staticSettings.moduleConfigurationProperties, hal.getEquipletName(), 
							dynamicSettings.mountPointX, dynamicSettings.mountPointY);
				} else if (dynamicSettings.attachedTo != null) {
					// this module is attached to another module
					ModuleIdentifier parentModuleIdentifier = dynamicSettings.attachedTo;
					insertSpace(parentModuleIdentifier);
					knowledgeDBClient.executeUpdateQuery(updateModuleToAttachedToModule, 
							staticSettings.moduleIdentifier.manufacturer, staticSettings.moduleIdentifier.typeNumber, staticSettings.moduleIdentifier.serialNumber, 
							staticSettings.moduleConfigurationProperties, hal.getEquipletName(), 
							parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber, 
							parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber);
				} else {
					throw new FactoryException("Module both attached to the mountplate and a module or neither");
				}
				
				// calibration
				for (CalibrationEntry calibrationEntry : staticSettings.calibrationData) {
					calibrationEntry.insertIntoDatabase(knowledgeDBClient);
				}
				knowledgeDBClient.getConnection().commit();
				return true;
			} catch (Exception ex) {
				Logger.log(LogSection.HAL_RECONFIG, LogLevel.ERROR, "Error occured while inserting module ", ex);
				knowledgeDBClient.getConnection().rollback();
				return false;
			} finally {
				knowledgeDBClient.getConnection().setAutoCommit(true);
			}
		} catch (SQLException ex) {
			return false;
		}
	}

	/**
	 * This method inserts space in the nested set tree for a module to be
	 * attached to the parentModule.
	 * 
	 * @param parentModuleIdentifier
	 * @throws KnowledgeException
	 */
	private void insertSpace(ModuleIdentifier parentModuleIdentifier) throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleLeft, parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber);
		knowledgeDBClient.executeUpdateQuery(insertSpaceInNestedTreeForModuleRight, parentModuleIdentifier.manufacturer, parentModuleIdentifier.typeNumber, parentModuleIdentifier.serialNumber);
	}

	/**
	 * This method removes a module from the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @return the static information of the module.
	 * @throws JSONException
	 * @throws ParseException 
	 */
	public StaticSettings removeModule(ModuleIdentifier moduleIdentifier) throws JSONException, ParseException {
		StaticSettings output = new StaticSettings();
		
		// collect all the data
		Module module = moduleFactory.getItemForIdentifier(moduleIdentifier);
		
		output.moduleIdentifier = module.getModuleIdentifier();
		output.moduleConfigurationProperties = module.getConfigurationProperties();
		
		output.moduleType = ModuleType.getSerializedModuleTypeByModuleTypeIdentifier(moduleIdentifier, knowledgeDBClient);

		ArrayList<CalibrationEntry> calibrationData = CalibrationEntry.getCalibrationDataForModule(moduleIdentifier, knowledgeDBClient);
		for (CalibrationEntry calibrationEntry : calibrationData) {
			output.calibrationData.add(calibrationEntry);
		}
		
		// remove all the data
		for (CalibrationEntry calibrationEntry : output.calibrationData) {
			calibrationEntry.removeFromDatabase(knowledgeDBClient);
		}
		
		knowledgeDBClient.executeUpdateQuery(removeModule, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber);
		removeSpace(moduleIdentifier);
		
		ModuleType.removeUnusedFromDatabase(knowledgeDBClient);
		
		// update the factories
		moduleFactory.checkCache();
		capabilityFactory.checkCache();

		return output;
	}

	/**
	 * This method removes space in the nested set tree for a module.
	 * 
	 * @param moduleIdentifier
	 *            is the identifier of the module to be removed.
	 * @throws KnowledgeException
	 */
	private void removeSpace(ModuleIdentifier moduleIdentifier) throws KnowledgeException {
		knowledgeDBClient.executeUpdateQuery(removeSpaceInNestedTreeForModuleLeft, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber);
		knowledgeDBClient.executeUpdateQuery(removeSpaceInNestedTreeForModuleRight, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber);
	}

	/**
	 * This method will return all the services supported by this equiplet.
	 * 
	 * @return
	 */
	public ArrayList<String> getAllSupportedServices() {
		ArrayList<String> services = new ArrayList<String>();

		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedServiceTypes, hal.getEquipletName());
		for (Row row : rows) {
			String serviceName = (String) row.get("serviceType");
			services.add(serviceName);
		}
		return services;
	}

}
