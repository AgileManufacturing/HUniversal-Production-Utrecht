/**
 * @file SoftwareData.java
 * @brief 
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
package rexos.mas;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.util.jar.JarFile;

/**
 * 
 **/
class SoftwareData {
	private boolean loaderNeedsRefresh = true;
	private long lastModified = 0;
	private SoftwareDescription description;
	private byte[] classData;
	private SoftwareClassLoader loader;
	
	public SoftwareData(SoftwareDescription description) {
		this.description = description;
	}
	
	public long getLastModified() {
		return lastModified;
	}

	public SoftwareDescription getDescription() {
		return description;
	}

	public void setLastModified(long lastModified) {
		this.lastModified = lastModified;
	}

	public void setDescription(SoftwareDescription description) {
		this.description = description;
		this.lastModified = 0;
		loaderNeedsRefresh = true;
	}
	
	public void setClassData(byte[] classData) {
		this.classData = classData;
		loaderNeedsRefresh = true;
	}
	
	public SoftwareClassLoader getLoader() throws IOException {
		updateClassData();
		if (loaderNeedsRefresh) {
			setLoader(new SoftwareClassLoader(SoftwareData.class.getClassLoader()));
			loader.registerClass(description.getClassName(), classData);
			loaderNeedsRefresh = false;
		}
		return loader;
	}

	public void setLoader(SoftwareClassLoader loader) {
		this.loader = loader;
	}
	
	private byte[] getClassDataFromJar(JarFile jar, String className) throws IOException {
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
			setClassData(getClassDataFromJar(jarFile, description.getClassName()));
			setLastModified(con.getLastModified());
		}
		
		jarFile.close();
	}
}
