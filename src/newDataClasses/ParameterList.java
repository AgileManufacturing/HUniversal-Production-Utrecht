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

import java.util.HashMap;

public class ParameterList{
	
	private HashMap<String, ParameterGroup> _parameterGroups;
	
	public ParameterList() {
		this._parameterGroups = new HashMap<String, ParameterGroup>();
	}
	
	public void AddParameterGroup(ParameterGroup parameterGroup) throws Exception {
		if(parameterGroup == null) throw new Exception("ParameterGroup can't be null");
		this._parameterGroups.put(parameterGroup.getName(), parameterGroup);
	}
	
	public ParameterGroup GetParameterGroup(String name) {
		return this._parameterGroups.get(name);
	}

}
