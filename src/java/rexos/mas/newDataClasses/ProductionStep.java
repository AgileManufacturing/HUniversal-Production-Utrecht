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
package rexos.mas.newDataClasses;

import java.io.Serializable;

import com.mongodb.BasicDBObject;

public class ProductionStep implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = -832835122145455883L;

	private int _requiredTimeSlots;
	private int _id;
	private long _capability;
	private ProductionStepStatus _status;
	
	private ParameterList _parameters;

	public ProductionStep() {
		this._parameters = new ParameterList();
		this._status = ProductionStepStatus.STATE_TODO;
	}
	
	public ProductionStep(int id, long capability,  ParameterList parameterList) {
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
	
	public ParameterList getParameterList() {
		return _parameters;
	}
	
	public BasicDBObject getParameterListAsDBObject(){
		BasicDBObject parameters = new BasicDBObject();
		for(String groupName : _parameters.getParametersGroups().keySet()){
			ParameterGroup group = _parameters.GetParameterGroup(groupName);
			BasicDBObject parametersDB = new BasicDBObject();
			for(Parameter parameter : group.getParameters()){
				parametersDB.put(parameter.getKey(), parameter.getValue());
			}
			parameters.put(group.getName(), parametersDB);
		}
		return parameters;
	}

	public long getCapability() {
		return _capability;
	}
	
}
