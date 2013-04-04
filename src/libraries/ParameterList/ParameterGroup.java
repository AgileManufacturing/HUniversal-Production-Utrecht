/**
 *
 * Project: product-agents
 *
 * Package: newDataClasses
 *
 * File: ParameterGroup.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package ParameterList;

import java.io.Serializable;
import java.util.Collection;
import java.util.HashMap;

public class ParameterGroup implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = 4985505923411671880L;

	private String _name;
	
	private HashMap<String, Parameter> _parameters;
	
	public ParameterGroup(String name) {
		this._name = name;
		this._parameters = new HashMap<String, Parameter>();
		
	}
	
	public ParameterGroup(Parameter parameter, String name) throws Exception {
		this(name);
		this.add(parameter);
	}
	
	public ParameterGroup(Parameter[] parameters, String name) throws Exception {
		this(name);
		this.add(parameters);
	}
	
	
	public String getName() {
		return this._name;
	}
	
	
	public void add(Parameter parameter) throws Exception {
		if(parameter == null) throw new Exception("Can't add a null parameter!");
		this._parameters.put(parameter.getKey(), parameter);
	}
	
	public void add(Parameter[] parameters) throws Exception {
		if(parameters == null) throw new Exception("Can't add null parameters!");
		for(Parameter p : parameters) {
			this.add(p);
		}
	}
	
	public Parameter getParameter(String key) {
		return this._parameters.get(key);
	}
	
	public Parameter[] getParameters() {
		Collection<Parameter> values = this._parameters.values();	
		return values.toArray(new Parameter[values.size()]);
	}
	
	/*
	 * Directly get for a parameter.
	 * No need to first retrieve the parameter object
	 */
	
	public String getParameterValue(String key) {
		return this._parameters.get(key).getValue();
	}
	
	public void setParameterValue(String key, String value) throws Exception {
			this._parameters.put(key, new Parameter(key, value));
	}
}