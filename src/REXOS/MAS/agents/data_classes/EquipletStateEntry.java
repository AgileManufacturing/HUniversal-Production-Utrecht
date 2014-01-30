/**
 * @file src/REXOS/MAS/agents/data_classes/EquipletStateEntry.java
 * @brief 
 * @date Created: 24 jun. 2013
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
package agents.data_classes;

import libraries.blackboard_client.data_classes.MongoSaveable;

import org.bson.types.ObjectId;

import com.mongodb.BasicDBObject;

/**
 * @author Peter Bonnema
 *
 */
public class EquipletStateEntry implements MongoSaveable {
	private ObjectId _id;
	private int equipletId;
	private EquipletMode equipletMode;
	private EquipletState equipletState;

	public EquipletStateEntry(int equipletId, EquipletMode equipletMode, EquipletState equipletState) {
		this.equipletId = equipletId;
		this.equipletMode = equipletMode;
		this.equipletState = equipletState;
	}
	
	public EquipletStateEntry(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		object.put("_id", _id);
		object.put("id", equipletId);
		object.put("mode", equipletMode.getValue());
		object.put("state", equipletState.getValue());
		return object;
	}

	@Override
	public void fromBasicDBObject(BasicDBObject object) throws IllegalArgumentException {
		try {
			BasicDBObject copy = (BasicDBObject) object.copy();
			_id = (ObjectId) copy.removeField("_id");
			equipletId = (int) copy.removeField("id");
			equipletMode = EquipletMode.getState((int) copy.removeField("mode"));
			equipletState = EquipletState.getState((int) copy.removeField("state"));
			
			if(!copy.isEmpty()) {
				throw new IllegalArgumentException();
			}
		} catch(ClassCastException | NullPointerException e) {
			e.printStackTrace();
			throw new IllegalArgumentException();
		}
	}

	/**
	 * @return the equipletId
	 */
	public int getEquipletId() {
		return equipletId;
	}

	/**
	 * @param equipletId the equipletId to set
	 */
	public void setEquipletId(int equipletId) {
		this.equipletId = equipletId;
	}

	/**
	 * @return the equipletMode
	 */
	public EquipletMode getEquipletMode() {
		return equipletMode;
	}

	/**
	 * @param equipletMode the equipletMode to set
	 */
	public void setEquipletMode(EquipletMode equipletMode) {
		this.equipletMode = equipletMode;
	}

	/**
	 * @return the equipletState
	 */
	public EquipletState getEquipletState() {
		return equipletState;
	}

	/**
	 * @param equipletState the equipletState to set
	 */
	public void setEquipletState(EquipletState equipletState) {
		this.equipletState = equipletState;
	}

	/**
	 * @return the id
	 */
	public ObjectId getId() {
		return _id;
	}
}
