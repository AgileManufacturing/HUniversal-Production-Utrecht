/**
 * @file rexos/mas/data/Part.java
 * @brief Data object for storing a part.
 * @date Created: 28 may 2013
 * 
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 * 
 **/
package rexos.mas.data;

import java.io.Serializable;

import com.mongodb.BasicDBObject;

/**
 * Data object for storing a part.
 */
public class Part implements Serializable, MongoSaveable {
	/**
	 * @var long serialVersionUID
	 *      The serialVersionUID for this class.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @var int id
	 *      The id of the part.
	 */
	private int id;

	/**
	 * @var int type
	 *      The type of the part.
	 */
	private int type;

	/**
	 * Basic constructor for a part with parameter type.
	 * 
	 * @param type The type of the part
	 */
	public Part(int type) {
		this.type = type;
		this.id = -1;
	}

	/**
	 * constructor for a part with parameters type and id.
	 * 
	 * @param type The type of the part
	 * @param id The id of the part
	 */
	public Part(int type, int id) {
		this.type = type;
		this.id = id;
	}

	/**
	 * constructor for a part accepting a BasicDBObject.
	 * 
	 * @param object The BasicDBObject for the part
	 */
	public Part(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * Setter for the type
	 * 
	 * @param type The new type for the part
	 */
	public void setType(int type) {
		this.type = type;
	}

	/**
	 * Getter for the type
	 * 
	 * @return The type of the part
	 */
	public int getType() {
		return type;
	}

	/**
	 * Setter for the id
	 * 
	 * @param id The new id for the part
	 */
	public void setId(int id) {
		this.id = id;
	}

	/**
	 * Getter for the id
	 * 
	 * @return The id of the part
	 */
	public int getId() {
		return id;
	}

	/**
	 * @see rexos.mas.data.IMongoSaveable#toBasicDBObject(com.mongodb.BasicDBObject)
	 **/
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("type", type);
		object.put("id", id);
		return object;
	}

	/**
	 * @see rexos.mas.data.IMongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 **/
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		BasicDBObject copy = (BasicDBObject) object.copy();
		try {
			type = (int) copy.remove("type");
			id = (int) copy.remove("id");
			if(!copy.isEmpty()){
				throw new IllegalArgumentException();
			}
		} catch(ClassCastException | NullPointerException e) {
			throw new IllegalArgumentException();
		}
	}

	/**
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format("Part [id=%s, type=%s]", id, type);
	}
}
