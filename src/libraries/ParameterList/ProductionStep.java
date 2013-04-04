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
package ParameterList;

public class ProductionStep {

	private int _requiredTimeSlots;
	
	private ParameterList _parameters;
	private Long _capability;

	public ProductionStep() {
		this._parameters = new ParameterList();
	}
	
	public ProductionStep(ParameterList parameterList) {
		this._parameters = parameterList;
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

	public Long getCapability() {
		return _capability;
	}

	public void setCapability(Long _capability) {
		this._capability = _capability;
	}
	
}
