/**
 * @file rexos/mas/equiplet_agent/ProductStepMessage.java
 * @brief Provides a message for the productstep blackboard
 * @date Created: 2013-04-03
 * 
 * @author Hessel Meulenbeld
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht.
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
 **/
package rexos.mas.equiplet_agent;

import jade.core.AID;

import java.io.Serializable;

import org.bson.types.ObjectId;

import rexos.libraries.log.Logger;
import rexos.mas.data.MongoSaveable;
import rexos.mas.data.ScheduleData;

import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;

/**
 * Implementation of a message for the productstep blackboard
 */
public class ProductStepMessage implements MongoSaveable, Serializable {
	private static final long serialVersionUID = 6752380093194239016L;

	/**
	 * @var Mongo ObjectId of the entry of this ProductStep in a blackboard.
	 */
	private ObjectId _id;

	/**
	 * @var AID productAgentId The AID of the productAgent linked to this
	 *      product step.
	 */
	private AID productAgentId;

	/**
	 * @var int type The type of the product step
	 */
	private int type;

	/**
	 * @var ParameterList parameters The parameterlist for this product step.
	 */
	private BasicDBObject parameters;

	/**
	 * @var int[] inputPartTypes step.
	 */
	private Integer[] inputPartTypes;

	/**
	 * @var int outputPartType
	 */
	private int outputPartType;

	/**
	 * @var StepStatusCode status The status for this product step.
	 */
	private StepStatusCode status;

	/**
	 * @var basicDBObject statusData The extra data provided by the status for
	 *      this product step.
	 */
	private BasicDBObject statusData;

	/**
	 * @var ScheduleData scheduleData The schedule for this product step.
	 */
	private ScheduleData scheduleData;

	/**
	 * The constructor for the product step entry.
	 * 
	 * @param productAgentId AID of the product agent linked to the product step
	 * @param type The type of the product step
	 * @param parameters The parameters for the product step
	 * @param inputPartTypes The input parts for the product step
	 * @param outputPartType The output parts for the product step
	 * @param status The status for the product step
	 * @param statusData The additional data for the status
	 * @param scheduleData The schedule data
	 */
	public ProductStepMessage(AID productAgentId, int type, BasicDBObject parameters,
			Integer[] inputPartTypes, int outputPartType, StepStatusCode status,
			BasicDBObject statusData, ScheduleData scheduleData) {
		this(null, productAgentId, type, parameters, inputPartTypes, outputPartType, status,
			statusData, scheduleData);
	}

	/**
	 * The constructor for the product step entry.
	 * 
	 * @param _id Mongo ObjectId of the entry of this ProductStep in a
	 *            blackboard.
	 * @param productAgentId AID of the product agent linked to the product step
	 * @param type The type of the product step
	 * @param parameters The parameters for the product step
	 * @param inputPartTypes The input parts for the product step
	 * @param outputPartType The output parts for the product step
	 * @param status The status for the product step
	 * @param statusData The additional data for the status
	 * @param scheduleData The schedule data
	 */
	public ProductStepMessage(ObjectId _id, AID productAgentId, int type,
			BasicDBObject parameters, Integer[] inputPartTypes, int outputPartType,
			StepStatusCode status, BasicDBObject statusData, ScheduleData scheduleData) {
		this._id = _id;
		this.productAgentId = productAgentId;
		this.type = type;
		this.parameters = parameters;
		this.inputPartTypes = inputPartTypes;
		this.outputPartType = outputPartType;
		this.status = status;
		this.statusData = statusData;
		this.scheduleData = scheduleData;
	}

	/**
	 * Constructor for a ProductStepMessage.
	 * 
	 * @param object
	 *            The BasicDBObject of which the ProductStepMessage has to be
	 *            builded.
	 */
	public ProductStepMessage(BasicDBObject object) {
		fromBasicDBObject(object);
	}

	/**
	 * @return the _id
	 */
	public ObjectId get_id() {
		return _id;
	}

	/**
	 * @param _id
	 *            the _id to set
	 */
	public void set_id(ObjectId _id) {
		this._id = _id;
	}

	/**
	 * @return the productAgentId
	 */
	public AID getProductAgentId() {
		return productAgentId;
	}

	/**
	 * @param productAgentId
	 *            the productAgentId to set
	 */
	public void setProductAgentId(AID productAgentId) {
		this.productAgentId = productAgentId;
	}

	/**
	 * @return the type
	 */
	public int getType() {
		return type;
	}

	/**
	 * @param type
	 *            the type to set
	 */
	public void setType(int type) {
		this.type = type;
	}

	/**
	 * @return the parameters
	 */
	public BasicDBObject getParameters() {
		return parameters;
	}

	/**
	 * @param parameters
	 *            the parameters to set
	 */
	public void setParameters(BasicDBObject parameters) {
		this.parameters = parameters;
	}

	/**
	 * @return the inputPartTypes
	 */
	public Integer[] getInputPartTypes() {
		return inputPartTypes;
	}

	/**
	 * @param inputPartTypes
	 *            the inputPartTypes to set
	 */
	public void setInputPartTypes(Integer[] inputPartTypes) {
		this.inputPartTypes = inputPartTypes;
	}

	/**
	 * @return the outputPartType
	 */
	public int getOutputPartType() {
		return outputPartType;
	}

	/**
	 * @param outputPartType
	 *            the outputPartType to set
	 */
	public void setOutputPartType(int outputPartType) {
		this.outputPartType = outputPartType;
	}

	/**
	 * @return the status
	 */
	public StepStatusCode getStatus() {
		return status;
	}

	/**
	 * @param status
	 *            the status to set
	 */
	public void setStatus(StepStatusCode status) {
		this.status = status;
	}

	/**
	 * @return the statusData
	 */
	public BasicDBObject getStatusData() {
		return statusData;
	}

	/**
	 * @param statusData
	 *            the statusData to set
	 */
	public void setStatusData(BasicDBObject statusData) {
		this.statusData = statusData;
	}

	/**
	 * @return the scheduleData
	 */
	public ScheduleData getScheduleData() {
		return scheduleData;
	}

	/**
	 * @param scheduleData
	 *            the scheduleData to set
	 */
	public void setScheduleData(ScheduleData scheduleData) {
		this.scheduleData = scheduleData;
	}

	/**
	 * Function for getting the BasicDBObject from this class.
	 * 
	 * @return the created BasicDBObject.
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject object = new BasicDBObject();
		if(_id != null)
			object.put("_id", _id);
		object.put("productAgentId", productAgentId.getName());
		object.put("type", type);
		object.put("parameters", parameters);
		object.put("inputPartTypes", inputPartTypes);
		object.put("outputPartType", outputPartType);
		object.put("status", status.toString());
		object.put("statusData", statusData);
		object.put("scheduleData", scheduleData.toBasicDBObject());
		return object;
	}

	/**
	 * Function to fill this class with a BasicDBObject.
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		_id = object.getObjectId("_id");
		try {
			productAgentId = new AID((String) (object.get("productAgentId")), AID.ISGUID);
		} catch(Exception e) {
			Logger.log(e);
		}
		type = object.getInt("type");
		parameters = (BasicDBObject) object.get("parameters");
		inputPartTypes = ((BasicDBList) object.get("inputPartTypes")).toArray(new Integer[0]);
		outputPartType = object.getInt("outputPartType", -1);
		status = StepStatusCode.valueOf(object.getString("status"));

		if(object.containsField("statusData")) {
			statusData = (BasicDBObject) object.get(statusData);
		} else {
			statusData = new BasicDBObject();
		}
		if(object.containsField("scheduleData")) {
			scheduleData = new ScheduleData((BasicDBObject) object.get("scheduleData"));
		} else {
			scheduleData = new ScheduleData();
		}
	}
}