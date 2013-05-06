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
	private static final long serialVersionUID = -3204119473462111867L;
	
	private Object _value;
	
	public Parameter() {
		this._value = null;
	}
	
	public Parameter(Object value) {
		this._value = value;
	}

	/**
	 * @return the _value
	 */
	public Object getValue() {
		return _value;
	}

	/**
	 * @param _value the _value to set
	 */
	public void setValue(Object value) {
		this._value = value;
	}
}
