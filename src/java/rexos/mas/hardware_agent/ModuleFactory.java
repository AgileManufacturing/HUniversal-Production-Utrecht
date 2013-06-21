/**
 * @file rexos/mas/hardware_agent/ModuleFactory.java
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

import java.util.ArrayList;
import java.util.Hashtable;

import rexos.libraries.dynamicloader.DynamicClassDescription;
import rexos.libraries.dynamicloader.DynamicClassFactory;
import rexos.libraries.dynamicloader.InstantiateClassException;
import rexos.libraries.knowledgedb_client.KeyNotFoundException;
import rexos.libraries.knowledgedb_client.KnowledgeDBClient;
import rexos.libraries.knowledgedb_client.KnowledgeException;
import rexos.libraries.knowledgedb_client.Queries;
import rexos.libraries.knowledgedb_client.Row;
import rexos.libraries.log.Logger;

/**
 * Creates and caches instances of the software required for modules.
 * This class makes sure that when a module is requested, an object of the latest version (according to the knowledge database) is returned.
 **/
public class ModuleFactory {
	/**
	 * @var Hashtable<Long, Module> moduleCache
	 * Hashtable containing instances of already instantiated modules.
	 **/
	private Hashtable<Integer, Module> moduleCache;
	
	/**
	 * @var DynamicClassFactory<Module> factory
	 * The DynamicClassFactory object that is used to load dynamic class data.
	 **/
	private DynamicClassFactory<Module> factory;
	
	/**
	 * @var ArrayList<ModuleUpdateListener> updateSubscribers
	 * The objects that have subscribed to software updates.
	 **/
	private ArrayList<ModuleUpdateListener> updateSubscribers;
	
	/**
	 * Initializes an empty ModuleFactory object.
	 **/
	public ModuleFactory() {
		factory = new DynamicClassFactory<Module>(Module.class);
		updateSubscribers = new ArrayList<ModuleUpdateListener>();
		moduleCache = new Hashtable<Integer, Module>();
	}
	
	/**
	 * Updates the moduleCache with the latest version retrieved from the knowledge database.
	 * @param moduleId The id of the module that should be updated.
	 **/
	private void updateModuleInCache(int moduleId) {
		try {
			KnowledgeDBClient knowledgeClient = KnowledgeDBClient.getClient();
		
			Row[] rows = knowledgeClient.executeSelectQuery(
					Queries.SOFTWARE_FOR_MODULE,
					new Object[]{moduleId});
			
			if (rows.length > 0) {
				DynamicClassDescription description = new DynamicClassDescription(
						new Long((Integer)rows[0].get("id")),
						(String)rows[0].get("name"),
						(String)rows[0].get("description"),
						(String)rows[0].get("class_name"),
						(String)rows[0].get("jar_location"));
				
				Module oldSoftware = moduleCache.get(moduleId);
				Module newSoftware = factory.createNewObjectIfOutdated(description, oldSoftware);
				moduleCache.put(moduleId, newSoftware);
				
				if (oldSoftware != newSoftware) {
					newSoftware.setId(moduleId);
					newSoftware.setName(description.getName());
					newSoftware.setModuleFactory(this);
					
					// Notify subscribers if the software was updated.
					// If no previous version exists, this is not considered an updated.
					if (oldSoftware != null) {
						for (ModuleUpdateListener sub : updateSubscribers) {
							sub.onModuleUpdate(moduleId, oldSoftware, newSoftware);
						}
					}
				}
			}
		} catch (InstantiateClassException | KnowledgeException | KeyNotFoundException e) {
			Logger.log(e);
		}
	}
	
	/**
	 * Returns an object representing the software required for the specified module.
	 * @param moduleId The module for which software should be loaded.
	 * @return Module object representing the software required for the specified module or null if no software could be loaded.
	 *
	 **/
	public Module getModuleById(int moduleId) {
		updateModuleInCache(moduleId);
		return moduleCache.get(moduleId);
	}
	
	/**
	 * Subscribe to software updates.
	 * The subscriber will be notified whenever a new version is loaded of a certain module.
	 * The first time software is loaded is not considered to be an update.
	 * @param listener The object expecting a callback on updates.
	 *
	 */
	public void subscribeToUpdates(ModuleUpdateListener listener) {
		if (!updateSubscribers.contains(listener)) {
			updateSubscribers.add(listener);
		}
	}
	
	/**
	 * Unsubscribe from software updates.
	 * @param listener The object that should be unsubscribed from updates.
	 *
	 **/
	public void unsubscribeToUpdates(ModuleUpdateListener listener) {
		updateSubscribers.remove(listener);
	}
}
