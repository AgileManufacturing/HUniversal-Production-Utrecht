/**
 *
 * Project: product-agents
 *
 * Package: newDataClasses
 *
 * File: ProductionStep.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

import java.io.Serializable;

import com.mongodb.BasicDBObject;

public class ProductionStep implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = -832835122145455883L;

	private int _requiredTimeSlots;
	private int _id;
	private int _capability;
	private ProductionStepStatus _status;
	private BasicDBObject _parameters;

	public ProductionStep() {
		this._parameters = new BasicDBObject();
		this._status = ProductionStepStatus.STATE_TODO;
	}
	
	public ProductionStep(int id, int capability, BasicDBObject parameterList) {
		this._id = id;
		this._capability = capability;
		this._parameters = parameterList;
		this._status = ProductionStepStatus.STATE_TODO;
	}
	
	public int getId(){
		return this._id;
	}
	
	public ProductionStepStatus getStatus(){
		return this._status;
	}
	
	public void setStatus(ProductionStepStatus status){
		this._status = status;
	}
	
	public void setRequiredTimeSlots(int timeSlots) {
		this._requiredTimeSlots = timeSlots;
	}
	
	public int getRequiredTimeSlots() {
		return this._requiredTimeSlots;
	}
	
	public BasicDBObject getParameters() {
		return _parameters;
	}

	public int getCapability() {
		return _capability;
	}
}
