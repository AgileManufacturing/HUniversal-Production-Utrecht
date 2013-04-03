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
package newDataClasses;

import java.io.Serializable;

public class Parameter implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = -7830428356814146610L;
	private String _key;
	private String _value;
	
	public Parameter(String key) throws Exception {
		if(key == null) throw new Exception("Key can't be null");
		this._key = key;
	}
	
	public Parameter(String key, String value) throws Exception {
		this(key);
		if(value == null) throw new Exception("Value can't be null");
		this._value = value;
	}
	
	/**
	 * @return the _key
	 */
	public String getKey() {
		return _key;
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
