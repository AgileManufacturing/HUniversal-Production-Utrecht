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
package newDataClasses;

import java.util.Collection;
import java.util.HashMap;

public class ParameterGroup {

	private String _name;
	
	private HashMap<String, Parameter> _parameters;
	
	public ParameterGroup() {
		this._parameters = new HashMap<String, Parameter>();
	}
	
	public ParameterGroup(Parameter parameter) throws Exception {
		this();
		this.add(parameter);
	}
	
	public ParameterGroup(Parameter[] parameters) throws Exception {
		this();
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
		return (Parameter[]) values.toArray();
	}
	
	
	
	
	
	
	
}
