/**
 *
 * Project: product-agents
 *
 * Package: newDataClasses
 *
 * File: ParameterList.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package newDataClasses;

import java.io.Serializable;
import java.util.HashMap;

public class ParameterList implements Serializable{
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -7394630866218200410L;
	private HashMap<String, ParameterGroup> _parameterGroups;
	
	public ParameterList() {
		this._parameterGroups = new HashMap<>();
	}
	
	public void AddParameterGroup(ParameterGroup parameterGroup) throws Exception {
		if(parameterGroup == null) throw new Exception("ParameterGroup can't be null");
		this._parameterGroups.put(parameterGroup.getName(), parameterGroup);
	}
	
	public ParameterGroup GetParameterGroup(String name) {
		return this._parameterGroups.get(name);
	}

	public HashMap<String, ParameterGroup> getParametersGroups(){
		return this._parameterGroups;
	}
	
}
