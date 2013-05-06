/**
 *
 * Project: product-agents
 *
 * Package: newDataClasses
 *
 * File: Parameter.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

import java.io.Serializable;

public class Parameter implements Serializable {
	private static final long serialVersionUID = 1L;
	
	private String _value;
	
	public Parameter() {
		this._value = null;
	}
	
	public Parameter(String value) {
		this._value = value;
	}

	/**
	 * @return the _value
	 */
	public String getValue() {
		return _value;
	}

	/**
	 * @param _value the _value to set
	 */
	public void setValue(String value) {
		this._value = value;
	}
}
