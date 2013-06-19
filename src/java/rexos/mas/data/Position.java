/**
 * @file rexos/mas/data/Position.java
 * @brief Data object for storing a 3d position.
 * @date Created: 22 apr. 2013
 * 
 * @author Peter Bonnema
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
 * Data object for storing a 3d position.
 * 
 */
public class Position implements Serializable, MongoSaveable {
	/**
	 * @var long serialVersionUID
	 * The serialVersionUID for this class.
**/
	private static final long serialVersionUID = 7125027379799391886L;

	/**
	 * @var double x
	 * The x coordinate.
**/
	private double x;
	
	/**
	 * @var double y
	 * The y coordinate.
**/
	private double y;
	
	/**
	 * @var double z
	 * The z coordinate
**/
	private double z;
	
	/**
	 * @var int relativeToPart
	 * The id of the part the coordinates are relative to.
**/
	private Part relativeToPart;

	/**
	 * Constructs a default Position object with all fields set to -1.
	 */
	public Position() {
		this(-1, -1, -1, null);
	}

	/**
	 * Constructs a Position object with the specified absolute x, y and z coordinates.
	 * @param x The x coordinate.
	 * @param y The y coordinate.
	 * @param z The z coordinate
	 */
	public Position(double x, double y, double z) {
		this(x, y, z, null);
	}

	/**
	 * Constructs a Position object with -1 x, y and z coordinates relative to the specified part.
	 * @param relativeToPart The id of the part to which the coordinates are relative.
	 */
	public Position(Part relativeToPart) {
		this(-1, -1, -1, relativeToPart);
	}

	/**
	 * Constructs a Position object with the specified x, y and z coordinates relative to the specified part.
	 * @param x The x coordinate.
	 * @param y The y coordinate.
	 * @param z The z coordinate
	 * @param relativeToPart The id of the part to which the coordinates are relative.
	 */
	public Position(double x, double y, double z, Part relativeToPart) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.relativeToPart = relativeToPart;
	}

	/**
	 * Constructs a Position object from the data contained within the specified BasicDBObject.
	 * @param object The BasicDBObject containing the required data.
	 */
	public Position(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/** 
	 * @see rexos.mas.data.IMongoSaveable#toBasicDBObject()
	 **/
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("x", x);
		object.put("y", y);
		object.put("z", z);
		if(relativeToPart != null){
			object.put("relativeToPart", relativeToPart.toBasicDBObject());
		}
		return object;
	}

	/** 
	 * @see rexos.mas.data.IMongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject)
	 **/
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		try {
			if(object.containsField("x")){
				x = object.getDouble("x");
			}else{
				x = -1;
			}
			if(object.containsField("y")){
				y = object.getDouble("y");
			}else{
				y = -1;
			}
			if(object.containsField("z")){
				z = object.getDouble("z");
			}else{
				z = -1;
			}
			if(object.containsField("relativeToPart")) {
				relativeToPart = new Part((BasicDBObject)object.get("relativeToPart"));
			}
		} catch(IllegalArgumentException | ClassCastException | NullPointerException e) {
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Returns the x coordinate for this Position.
	 * @return The x coordinate.
	 */
	public double getX() {
		return x;
	}

	/**
	 * Sets the x coordinate for this Position.
	 * @param x The x coordinate that should be set.
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * Returns the y coordinate for this Position.
	 * @return The y coordinate.
	 */
	public double getY() {
		return y;
	}

	/**
	 * Sets the y coordinate for this Position.
	 * @param y The y coordinate that should be set.
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * Returns the z coordinate for this Position.
	 * @return The z coordinate.
	 */
	public double getZ() {
		return z;
	}

	/**
	 * Sets the z coordinate for this Position.
	 * @param z The z coordinate that should be set.
	 */
	public void setZ(double z) {
		this.z = z;
	}

	/**
	 * Returns the partID of the relative part.
	 * @return The partID of the relative part.
	 */
	public Part getRelativeToPart() {
		return relativeToPart;
	}

	/**
	 * Sets the partID of the relative part.
	 * @param relativeToPart The partID of the relative part.
	 */
	public void setRelativeToPart(Part relativeToPart) {
		this.relativeToPart = relativeToPart;
	}

	/**
	 * @see java.lang.Object#toString()
	 **/
	@Override
	public String toString() {
		return String.format("Position [x=%s, y=%s, z=%s, relativeToPart=%s]", x, y, z, relativeToPart);
	}
}
