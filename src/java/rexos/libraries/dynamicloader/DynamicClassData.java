/**
 * @file DynamicClassData.java
 * @brief Contains all data required to instantiate an object of a class.
 * @date Created: 13 apr. 2013
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

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.util.jar.JarFile;

/**
 * Contains all data required to instantiate an object of a class.
 **/
class DynamicClassData {
	/**
	 * @var boolean loaderNeedsRefresh
	 * Indicates whether or not the class data has changed since creating the last SoftwareClassLoader.
	 **/
	private boolean classDataChanged = true;
	
	/**
	 * @var long lastModified
	 * Timestamp indicating when the class data was last updated.
	 **/
	private long lastModified = 0;
	
	/**
	 * @var SoftwareDescription description
	 * Description of the class represented by this SoftwareData object.
	 **/
	private DynamicClassDescription description;
	
	/**
	 * @var byte[] classData
	 * The classdata of the class represented by this SoftwareData object.
	 **/
	private byte[] classData;
	
	/**
	 * @var SoftwareClassLoader loader
	 * Current instance of the DynamicClassLoader for this class.
	 **/
	private DynamicClassLoader loader;
	
	/**
	 * Construct an object for the given description.
	 * @param description DynamicClassDescription describing the class.
	 **/
	public DynamicClassData(DynamicClassDescription description) {
		this.description = description;
	}
	
	/**
	 * Returns the DynamicClassDescription for this object.
	 * @return the DynamicClassDescription for this object.
	 **/
	public DynamicClassDescription getDescription() {
		return description;
	}

	
	/**
	 * Returns the last modified timestamp.
	 * @return the last modified timestamp.
	 **/
	public long getLastModified() {
		return lastModified;
	}

	/**
	 * Attempts to return a classloader for the latest classdata.
	 * If updating the class data fails for whatever reason, a loader for the previous version is returned if available.
	 * @return A SoftwareClassLoader for the class represented by this object.
	 * @throws InstantiateClassException Retrieving the class data failed.
	 **/
	public DynamicClassLoader getLoader() throws InstantiateClassException {
		// Attempt to update the class data.
		// If an error occurs but a previous version is in cache, use that instead.
		try {
			updateClassData();
		} catch (IOException ex) {
			if (getLastModified() != 0) {
				throw new InstantiateClassException("Failed to retrieve software.", ex);
			}
		}
		if (classDataChanged) {
			setLoader(new DynamicClassLoader(DynamicClassData.class.getClassLoader()));
			loader.registerClass(description.getClassName(), classData);
			classDataChanged = false;
		}
		return loader;
	}
	
	/**
	 * Sets the byte[] containing class data for this object.
	 * The array is not copied and should be preallocated.
	 * @param classData The class data that should be used.
	 **/
	private void setClassData(byte[] classData) {
		this.classData = classData;
		classDataChanged = true;
	}
	
	/**
	 * Sets the description that will be used for this object.
	 * @param description The description that will be used for this object.
	 **/
	public void setDescription(DynamicClassDescription description) {
		if (!description.equals(this.description)) {
			this.lastModified = 0;
			classDataChanged = true;
		}
		this.description = description;
	}

	/**
	 * Sets the last modified timestamp.
	 * @param lastModified The last time the class data was updated.
	 **/
	private void setLastModified(long lastModified) {
		this.lastModified = lastModified;
	}
	
	/**
	 * Sets the loader for this object.
	 * @param loader DynamicClassLoader capable of instantiating an object for the lastest version of the represented class.
	 **/
	private void setLoader(DynamicClassLoader loader) {
		this.loader = loader;
	}
	
	/**
	 * Extract data for a given class from the specified jar file.
	 * @param jar JarFile containing the class data.
	 * @param className Fully qualified name of the class that should be extracted.
	 * @return Byte array containing the data for the specified class.
	 * @throws IOException Reading from the JarFile failed.
	 **/
	private byte[] extractClassDataFromJar(JarFile jar, String className) throws IOException {
		// Convert classname to  in jar.
		String pathInJar = className.replace('.', '/').concat(".class");
		InputStream inStream = jar.getInputStream(jar.getJarEntry(pathInJar));
		
		ByteArrayOutputStream buffer = new ByteArrayOutputStream();
		int b;
		while ((b = inStream.read()) != -1) {
			buffer.write(b);
		}
		
		return buffer.toByteArray();
	}
	
	/**
	 * Attempts to update the stored class data to the latest version if available.
	 * @throws IOException Retrieving an updated version of the class data failed.
	 **/
	private void updateClassData() throws IOException {
		JarFile jarFile = null;
		URL jarLocationURL = new URL(description.getJarLocation());
		URLConnection con = jarLocationURL.openConnection();
		
		// Set last modified date so new data is only obtained when something has actually changed.
		con.setIfModifiedSince(lastModified);
		con.connect();
		InputStream inputStream = con.getInputStream();
		ByteArrayOutputStream buffer = new ByteArrayOutputStream();
		
		int data;
		while ((data = inputStream.read()) != -1) {
			buffer.write(data);
		}
		inputStream.close();
		
		if (buffer.size() > 1) {
			int lastSlashPos = jarLocationURL.getPath().lastIndexOf('/');
			String jarFileName = jarLocationURL.getPath().substring(lastSlashPos > -1 ? lastSlashPos + 1 : 0);
			File file = new File(jarFileName);
			file.deleteOnExit();
			FileOutputStream fos = new FileOutputStream(file);
			fos.write(buffer.toByteArray());
			fos.close();
			jarFile = new JarFile(file);
		}
		
		if (jarFile != null) {
			setClassData(extractClassDataFromJar(jarFile, description.getClassName()));
			setLastModified(con.getLastModified());
			jarFile.close();
		}
	}
}
