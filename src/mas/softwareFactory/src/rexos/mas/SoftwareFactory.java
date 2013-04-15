/**
 * @file SoftwareFactory.java
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
package rexos.mas;

import java.util.Hashtable;

/**
 * Generic class for loading class data from a remote server based on a description.
 **/
public class SoftwareFactory<T> {
	/**
	 * @var Hashtable<Long, SoftwareData> softwareCache
	 * A cache holding the SofwareData for each id.
	 **/
	private Hashtable<Long, SoftwareData>softwareCache;
	
	/**
	 * Constructs a new SoftwareFactory.
	 **/
	public SoftwareFactory() {
		softwareCache = new Hashtable<Long, SoftwareData>();
	}
	
	/**
	 * Returns a SoftwareClassLoader for the given description.
	 * @param description SoftwareDescription containing the relevant information for the software.
	 * @return A SoftwareClassLoader that is able to create an object for the given description.
	 * @throws SoftwareLoadException No loader could be created.
	 **/
	private SoftwareClassLoader getClassLoader(SoftwareDescription description) throws SoftwareLoadException {
		SoftwareData entry = softwareCache.get(description.getId());
		if (entry == null) {
			entry = new SoftwareData(description);
			softwareCache.put(description.getId(), entry);
		}
		
		return entry.getLoader();
	}
	
	/**
	 * Attempts to instantiate an object of the class specified in the description.
	 * @param description SoftwareDescription containing the relevant information for the software.
	 * @return An object of the class specified in the description.
	 * @throws SoftwareLoadException The object of the specified class could not be instantiated.
	 *
	 **/
	public T createObjectFromDescription(SoftwareDescription description) throws SoftwareLoadException {
		try {
			SoftwareClassLoader loader = getClassLoader(description);
			Class<?> cls = loader.loadClass(description.getClassName());
			
			@SuppressWarnings("unchecked")
			T obj = (T)cls.newInstance();
			return obj;
		} catch (ClassNotFoundException | InstantiationException | IllegalAccessException ex) {
			throw new SoftwareLoadException("Failed to instantiate object of class " + description.getClassName(), ex);
		}
	}
}
