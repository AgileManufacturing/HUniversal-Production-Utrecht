/**
 * @file rexos/libraries/dynamicloader/DynamicClassLoader.java
 * @brief Custom class loader.
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
package rexos.libraries.dynamicloader;

import java.util.HashMap;

/**
 * Custom class loader.
 **/
public class DynamicClassLoader extends ClassLoader {
	
	/**
	 * @var HashMap<String,byte[]> registeredClasses
	 * The classes that have been registered for loading for this object.
	 **/
	HashMap<String, byte[]> registeredClasses;
	
	/**
	 * Constructs a new DynamicClassLoader object.
	 * @param parent The parent class loader that should be used.
	 **/
	public DynamicClassLoader(ClassLoader parent) {
		super(parent);
		registeredClasses = new HashMap<String, byte[]>();
	}
	
	/**
	 * Checks whether or not the specified class has been registered for loading.
	 * @param name The name of the class.
	 * @return Whether or not the specified class has been registered for loading.
	 **/
	public boolean isClassRegistered(String name) {
		return registeredClasses.containsKey(name);
	}
	
	/**
	 * Registers a class for loading with this class loader object.
	 * @param name The name of the class.
	 * @param data A byte[] containing the definition of the class. (i.e. contents of the .class file)
	 **/
	public void registerClass(String name, byte[] data) {
		registeredClasses.put(name, data);
	}
	
	/**
	 * Attempts to load the class corresponding to the given name.
	 * @param name The name of the class
	 * @return The resulting Class object.
	 * @see java.lang.ClassLoader#loadClass(java.lang.String)
	 **/
	public Class<?> loadClass(String name) throws ClassNotFoundException {
		if (!registeredClasses.containsKey(name)) {
			return super.loadClass(name);
		}
		
		byte[] classData = registeredClasses.get(name);
		return defineClass(name, classData, 0, classData.length);
	}
}
