/**
 * @file src/REXOS/MAS/libraries/dynamicloader/DynamicClassFactory.java
 * @brief Generic class for loading class data from a remote server based on a description.
 * @date Created: 12 apr. 2013
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
package libraries.dynamicloader;

import java.util.Hashtable;

/**
 * Generic class for loading class data from a remote server based on a description.
 **/
public class DynamicClassFactory<T> {
	/**
	 * @var Hashtable<Long, DynamicClassData> softwareCache
	 * A cache holding the DynamicClassData for each id.
	 **/
	private Hashtable<Integer, DynamicClassData>softwareCache;
	
	private JarFileLoader jarFileLoader;
	/**
	 * Constructs a new DynamicClassFactory.
	 **/
	public DynamicClassFactory(JarFileLoader jarFileLoader) {
		this.jarFileLoader = jarFileLoader;
		softwareCache = new Hashtable<Integer, DynamicClassData>();
	}
	
	/**
	 * Returns a DynamicClassLoader for the given description.
	 * @param description DynamicClassDescription containing the relevant information for the software.
	 * @return A DynamicClassLoader that is able to create an object for the given description.
	 * @throws InstantiateClassException No loader could be created.
	 * @throws JarFileLoaderException 
	 **/
	private DynamicClassLoader getClassLoader(DynamicClassDescription description) throws InstantiateClassException, JarFileLoaderException {
		DynamicClassData entry = softwareCache.get(description.getId());
		if (entry == null) {
			entry = new DynamicClassData(description, jarFileLoader);
			softwareCache.put(description.getId(), entry);
		}
		// Update the description.
		entry.setDescription(description);
		
		return entry.getLoader();
	}
	
	/**
	 * Use the given loader to instantiate an object of the specified class.
	 * @param loader The loader that should be used to instantiate the object.
	 * @param className The className of the class that should be instantiated.
	 * @return An object of the specified class.
	 * @throws InstantiateClassException The object of the specified class could not be instantiated.
	 *
	 **/
	private Class<T> getClassWithLoader(DynamicClassLoader loader, String className) throws InstantiateClassException {
		try {
			return (Class<T>) loader.loadClass(className);
		} catch (ClassNotFoundException ex) {
			throw new InstantiateClassException("Failed to instantiate object of class " + className, ex);
		} catch (ClassCastException ce) {
			throw new InstantiateClassException("The described class " + className + " is not castable to "/*
					+ String.class.getName()*/, ce);
		}
	}
	
	/**
	 * Attempts to instantiate an object of the class specified in the description.
	 * @param description DynamicClassDescription containing the relevant information for the software.
	 * @return An object of the class specified in the description.
	 * @throws InstantiateClassException The object of the specified class could not be instantiated.
	 * @throws JarFileLoaderException 
	 *
	 **/
	public Class<T> getClassFromDescription(DynamicClassDescription description) throws InstantiateClassException, JarFileLoaderException {
			DynamicClassLoader loader = getClassLoader(description);
			return getClassWithLoader(loader, description.getClassName());
	}
	
	/**
	 * Checks of the given obj is up to date according to the specified description.
	 * If it's not, a new object will be constructed.
	 * @param description DynamicClassDescription containing the relevant information for the software.
	 * @param obj The object that should be checked.
	 * @return An object of type T that is up to date according to the given description.
	 * @throws InstantiateClassException The object of the specified class could not be instantiated.
	 **/
/*	public T gObjectIfOutdated(DynamicClassDescription description, T obj) throws InstantiateClassException {
		T objToReturn = obj;
		DynamicClassLoader loader = getClassLoader(description);
		if (obj == null || !obj.getClass().getClassLoader().equals(loader)) {
			objToReturn = createObjectWithLoader(loader, description.getClassName());
		}
		return objToReturn;
	}*/
}
