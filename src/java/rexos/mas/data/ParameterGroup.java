/**
 * @file ParameterGroup.java
 * @brief Class where parameters can be added to a parametergroup and their
 *        value can be set.
 * @date Created: 02-04-2013
 * 
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.data;

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

	public ParameterGroup(String name){
		this._name = name;
		this._parameters = new HashMap<>();
	}

	public ParameterGroup(Parameter parameter, String name) throws Exception{
		this(name);
		this.add(parameter);
	}

	public ParameterGroup(Parameter[] parameters, String name) throws Exception{
		this(name);
		this.add(parameters);
	}

	public String getName(){
		return this._name;
	}

	public void add(Parameter parameter) throws Exception{
		if (parameter == null)
			throw new Exception("Can't add a null parameter!");
		this._parameters.put(parameter.getParamKey(), parameter);
	}

	public void add(Parameter[] parameters) throws Exception{
		if (parameters == null)
			throw new Exception("Can't add null parameters!");
		for(Parameter p : parameters){
			this.add(p);
		}
	}

	public Parameter getParameter(String key){
		return this._parameters.get(key);
	}

	public Parameter[] getParameters(){
		Collection<Parameter> values = this._parameters.values();
		return values.toArray(new Parameter[values.size()]);
	}

	/*
	 * Directly get for a parameter. No need to first retrieve the parameter
	 * object
	 */
	public String getParameterValue(String key){
		return this._parameters.get(key).getParamValue();
	}

	public void setParameterValue(String key, String value) throws Exception{
		this._parameters.put(key, new Parameter(key, value));
	}
}
