/**
 * @file ModuleFactory.java
 * @brief Creates and caches instances of the software required for modules.
 * @date Created: 22 apr. 2013
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package rexos.mas.hardware_agent;

import java.sql.ResultSet;
import java.sql.SQLException;
import java.util.Hashtable;

import rexos.libraries.dynamicloader.DynamicClassDescription;
import rexos.libraries.dynamicloader.DynamicClassFactory;
import rexos.libraries.dynamicloader.InstantiateClassException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;

/**
 * Creates and caches instances of the software required for modules.
 * This class makes sure that when a module is requested, an object of the latest version (according to the knowledge database) is returned.
 **/
public class ModuleFactory {
	/**
	 * @var Hashtable<Long, Module> moduleCache
	 * Hashtable containing instances of already instantiated modules.
	 **/
	private Hashtable<Long, Module> moduleCache;
	
	/**
	 * @var DynamicClassFactory<Module> factory
	 * The DynamicClassFactory object that is used to load dynamic class data.
	 **/
	private DynamicClassFactory<Module> factory;
	
	/**
	 * Initializes an empty ModuleFactory object.
	 **/
	public ModuleFactory() {
		factory = new DynamicClassFactory<Module>(Module.class);
		moduleCache = new Hashtable<Long, Module>();
	}
	
	/**
	 * Updates the moduleCache with the latest version retrieved from the knowledge database.
	 * @param moduleId The id of the module that should be updated.
	 **/
	private void updateModuleInCache(long moduleId) {
		KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
		try {
			ResultSet result = knowledgeClient.executeSelectQuery(
					Queries.SOFTWARE_FOR_MODULE,
					new Object[]{moduleId});
			
			if (result.next()) {
				Row row = new Row(result);
				DynamicClassDescription description = DynamicClassDescription.createFromRow(row);
				moduleCache.put(
						moduleId,
						factory.createNewObjectIfOutdated(
								description,
								moduleCache.get(moduleId)));
			}
		} catch (SQLException | InstantiateClassException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Returns an object representing the software required for the specified module.
	 * @param moduleId The module for which software should be loaded.
	 * @return Module object representing the software required for the specified module or null if no software could be loaded.
	 *
	 **/
	public Module getModuleById(long moduleId) {
		updateModuleInCache(moduleId);
		return moduleCache.get(moduleId);
	}
}
