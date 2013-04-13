/**
 * @file SoftwareDescription.java
 * @brief 
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

import rexos.libraries.knowledge.Row;

/**
 * 
 **/
public class SoftwareDescription {
	private long ID;
	private String name;
	private String description;
	private String className;
	private String jarLocation;
	
	public SoftwareDescription(long ID, String name, String description, String className, String jarLocation) {
		this.ID = ID;
		this.name = name;
		this.description = description;
		this.className = className;
		this.jarLocation = jarLocation;
	}
	
	public static SoftwareDescription createFromRow(Row row) {
		return new SoftwareDescription(
				(long)row.get("id"),
				(String)row.get("name"),
				(String)row.get("description"),
				(String)row.get("class_name"),
				(String)row.get("jar_location"));
	}
	
	public long getID() {
		return ID;
	}

	public String getName() {
		return name;
	}

	public String getDescription() {
		return description;
	}

	public String getClassName() {
		return className;
	}

	public String getJarLocation() {
		return jarLocation;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (int) (ID ^ (ID >>> 32));
		result = prime * result
				+ ((className == null) ? 0 : className.hashCode());
		result = prime * result
				+ ((description == null) ? 0 : description.hashCode());
		result = prime * result
				+ ((jarLocation == null) ? 0 : jarLocation.hashCode());
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		SoftwareDescription other = (SoftwareDescription) obj;
		if (ID != other.ID)
			return false;
		if (className == null) {
			if (other.className != null)
				return false;
		} else if (!className.equals(other.className))
			return false;
		if (description == null) {
			if (other.description != null)
				return false;
		} else if (!description.equals(other.description))
			return false;
		if (jarLocation == null) {
			if (other.jarLocation != null)
				return false;
		} else if (!jarLocation.equals(other.jarLocation))
			return false;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		return true;
	}

}
