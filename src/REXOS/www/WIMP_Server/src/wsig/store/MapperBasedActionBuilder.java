/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig.store;

import jade.content.AgentAction;
import jade.content.abs.AbsObject;
import jade.content.onto.Ontology;

import java.lang.reflect.Method;
import java.util.Vector;

import wsig.wsdl.WSDLUtils;

public class MapperBasedActionBuilder extends ActionBuilder {
	
	private Method method;
	private Object mapperObj;
	private String[] methodParameterNames;

	public MapperBasedActionBuilder(Object mapperObj, Method method, Ontology onto, String ontoActionName) {
		super(onto, ontoActionName);
		
		this.method = method;
		this.mapperObj = mapperObj;
		this.methodParameterNames = WSDLUtils.getParameterNames(method);
	}

	public AgentAction getAgentAction(Vector<ParameterInfo> soapParams) throws Exception {
		
		Object[] parameterValues = new Object[0];
		String parameterList = "";

		// Prepare mapper parameter
		if (soapParams != null) {
			
	        // If the mapper class is a Java dynamic proxy is not possible to have 
			// the name of the parameters of the methods (methodParameterNames == null)
			// Use SOAP parameters as master and apply it in the order of vector   
			// See: WSDLGeneratorUtils.getParameterNames(method)
			if (methodParameterNames == null) {
				parameterValues = new Object[soapParams.size()];
				for (int i = 0; i < soapParams.size(); i++) {
					ParameterInfo pi = soapParams.get(i);
					AbsObject absValue = pi.getValue();
					Object javaValue = getOntology().toObject(absValue);
					parameterValues[i] = javaValue;
					parameterList += pi.getSchema().getTypeName()+",";
				}
			} else {
				parameterValues = new Object[methodParameterNames.length];
				for (int i = 0; i < methodParameterNames.length; i++) {
					try {
						ParameterInfo pi = getSoapParamByName(soapParams, methodParameterNames[i]);
						AbsObject absValue = pi.getValue();
						Object javaValue = getOntology().toObject(absValue);
						parameterValues[i] = javaValue;
						parameterList += pi.getSchema().getTypeName()+",";
					} catch(Exception e) {
						log.error("Method "+method.getName()+", mandatory param "+methodParameterNames[i]+" not found in soap request");
						throw e;
					}
				}
			}
			if (parameterList.endsWith(",")) {
				parameterList = parameterList.substring(0, parameterList.length()-1);
			}
		}
		
		AgentAction actionObj = null;
		try {
			// Invoke mapper method
			actionObj = (AgentAction)method.invoke(mapperObj, parameterValues);
			log.debug("Invoked method "+method.getName()+"("+parameterList+") in mapper");

		} catch(Exception e) {
			log.error("Method "+method.getName()+"("+parameterList+") error invocation");
			throw e;
		}
		
		return actionObj;
	}
	
	private ParameterInfo getSoapParamByName(Vector<ParameterInfo> soapParams, String methodParamName) throws Exception {
		for (ParameterInfo param : soapParams) {
			if (param.getName().equalsIgnoreCase(methodParamName)) {
				return param;
			}
		}
		
		throw new Exception();
	}
}
