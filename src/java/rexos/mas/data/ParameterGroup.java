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
package rexos.mas.data;

import java.io.Serializable;
import java.util.HashMap;

import com.mongodb.BasicDBObject;

/**
 * @author Peter
 * 
 */
public class ParameterGroup extends Parameter implements IMongoSaveable, Serializable {
	private static final long serialVersionUID = 1L;
	
	/**
	 * 
	 */
	private HashMap<String, Parameter> _parameters;

	/**
	 * 
	 */
	public ParameterGroup() {
		_parameters = new HashMap<String, Parameter>();
	}

	/**
	 * 
	 */
	public ParameterGroup(BasicDBObject object) {
		_parameters = new HashMap<String, Parameter>();
		fromBasicDBObject(object);
	}

	/**
	 * @param name
	 * @param parameter
	 */
	public void addParameter(String name, Parameter parameter) {
		_parameters.put(name, parameter);
	}

	/**
	 * @param name
	 * @return
	 */
	public Parameter getParameter(String name) {
		return _parameters.get(name);
	}

	/**
	 * @param name
	 * @return
	 */
	public Parameter removeParameter(String name) {
		return _parameters.remove(name);
	}

	/**
	 * @return
	 */
	public HashMap<String, Parameter> getParameters() {
		return (HashMap<String, Parameter>) _parameters.clone();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see rexos.mas.data.IMongoSaveable#toBasicDBObject()
	 */
	@Override
	public BasicDBObject toBasicDBObject() {
		BasicDBObject dbObject = new BasicDBObject();
		for (String name : _parameters.keySet()) {
			Parameter parameter = _parameters.get(name);
			if (parameter instanceof ParameterGroup) {
				dbObject.append(name,
						((ParameterGroup) parameter).toBasicDBObject());
			} else {
				dbObject.append(name, parameter.getValue());
			}
		}
		return dbObject;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * rexos.mas.data.IMongoSaveable#fromBasicDBObject(com.mongodb.BasicDBObject
	 * )
	 */
	@Override
	public void fromBasicDBObject(BasicDBObject object) {
		for (String name : object.keySet()) {
			Object parameter = object.get(name);
			if (parameter instanceof BasicDBObject) {
				_parameters.put(name, new ParameterGroup(
						(BasicDBObject) parameter));
			} else {
				_parameters.put(name, new Parameter((String) object.get(name)));
			}
		}
	}
}
