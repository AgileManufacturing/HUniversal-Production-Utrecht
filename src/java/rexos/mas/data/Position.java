/**
 * @file Position.java
 * @brief 
 * @date Created: 22 apr. 2013
 *
 * @author Peter Bonnema
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
 * 
 **/
package rexos.mas.data;

import java.io.Serializable;

import com.mongodb.BasicDBObject;
import com.mongodb.BasicDBObjectBuilder;

/**
 * @author Peter
 * 
 */
public class Position implements Serializable, IMongoSaveable {
	private static final long serialVersionUID = 1L;

	private long x, y, z;
	private long relativeToPartId;

	/**
	 * 
	 */
	public Position() {
		this(-1, -1, -1, -1);
	}

	/**
	 * @param object
	 */
	public Position(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * @param x
	 * @param y
	 * @param z
	 */
	public Position(long x, long y, long z) {
		this(x, y, z, -1);
	}

	/**
	 * @param x
	 * @param y
	 * @param z
	 * @param relativeToPartId
	 */
	public Position(long x, long y, long z, long referencePartId) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.relativeToPartId = referencePartId;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.data.IMongoSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		return (BasicDBObject) BasicDBObjectBuilder.start()
				.add("x", x)
				.add("y", y)
				.add("z", z)
				.add("relativeToPartId", relativeToPartId).get();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.data.IMongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject
	 * )
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		x = object.getLong("x");
		y = object.getLong("y");
		z = object.getLong("z");
		relativeToPartId = object.getLong("relativeToPartId");
	}

	/**
	 * @return the x
	 */
	public long getX() {
		return x;
	}

	/**
	 * @param x
	 *            the x to set
	 */
	public void setX(long x) {
		this.x = x;
	}

	/**
	 * @return the y
	 */
	public long getY() {
		return y;
	}

	/**
	 * @param y
	 *            the y to set
	 */
	public void setY(long y) {
		this.y = y;
	}

	/**
	 * @return the z
	 */
	public long getZ() {
		return z;
	}

	/**
	 * @param z
	 *            the z to set
	 */
	public void setZ(long z) {
		this.z = z;
	}

	/**
	 * @return the relativeToPartId
	 */
	public long getRelativeToPartId() {
		return relativeToPartId;
	}

	/**
	 * @param relativeToPartId
	 *            the relativeToPartId to set
	 */
	public void setRelativeToPartId(long relativeToPartId) {
		this.relativeToPartId = relativeToPartId;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return String.format("Position [x=%s, y=%s, z=%s, relativeToPartId=%s]",
				x, y, z, relativeToPartId);
	}
}
