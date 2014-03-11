/**
 * @file src/REXOS/MAS/libraries/dynamicloader/DynamicClassDescription.java
 * @brief Contains all information about a piece of software.
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

/**
 * Contains all information about a piece of software.
 **/
public class DynamicClassDescription {
	/**
	 * @var long id
	 * The ID (corresponding to the Knowledge Database) of this piece of software.
	 **/
	private int id;
	
	/**
	 * @var String className
	 * The (fully distinguished) name of the class that represents this object.
	 **/
	private String className;
	
	/**
	 * Constructs a new DynamicClassDescription object.
	 * @param ID The ID (corresponding to the Knowledge Database) of this piece of software.
	 * @param name The name of this piece of software.
	 * @param description A description of the software.
	 * @param className The (fully distinguished) name of the class that represents this object.
	 * @param jarLocation The location of the jar containing the data for the representing class.
	 **/
	public DynamicClassDescription(int id, String className) {
		this.id = id;
		this.className = className;
	}

	/**
	 * Returns the ID (corresponding to the Knowledge Database) of this piece of software.
	 * @return The ID (corresponding to the Knowledge Database) of this piece of software.
	 **/
	public int getId() {
		return id;
	}

	/**
	 * Returns the (fully distinguished) name of the class that represents this object.
	 * @return The (fully distinguished) name of the class that represents this object.
	 **/
	public String getClassName() {
		return className;
	}

	/**
	 * @see java.lang.Object#hashCode()
	 **/
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (int) (id ^ (id >>> 32));
		result = prime * result
				+ ((className == null) ? 0 : className.hashCode());
		return result;
	}

	/**
	 * @see java.lang.Object#equals(java.lang.Object)
	 **/
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DynamicClassDescription other = (DynamicClassDescription) obj;
		if (id != other.id)
			return false;
		if (className == null) {
			if (other.className != null)
				return false;
		} else if (!className.equals(other.className))
			return false;
		return true;
	}

}
