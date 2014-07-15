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

import jade.content.onto.Ontology;
import jade.core.AID;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import javax.wsdl.Definition;

import org.uddi4j.util.ServiceKey;

public class WSIGService {

	private String serviceName;
	private String servicePrefix;
	private AID aid;
	private Ontology onto;
	private Definition wsdlDefinition;
	private ServiceKey uddiServiceKey;
	private Class mapperClass;
	private Map<String,ActionBuilder> actionsBuilder = new HashMap<String,ActionBuilder>();
	
	public AID getAid() {
		return aid;
	}
	public void setAid(AID aid) {
		this.aid = aid;
	}
	public Ontology getOnto() {
		return onto;
	}
	public void setOnto(Ontology onto) {
		this.onto = onto;
	}
	public String getServiceName() {
		return serviceName;
	}
	public void setServiceName(String serviceName) {
		this.serviceName = serviceName;
	}
	public Definition getWsdlDefinition() {
		return wsdlDefinition;
	}
	public void setWsdlDefinition(Definition wsdlDefinition) {
		this.wsdlDefinition = wsdlDefinition;
	}
	public ServiceKey getUddiServiceKey() {
		return uddiServiceKey;
	}
	public void setUddiServiceKey(ServiceKey uddiServiceKey) {
		this.uddiServiceKey = uddiServiceKey;
	}
	public Collection<String> getOperations() {
		return actionsBuilder.keySet();
	}
	public Class getMapperClass() {
		return mapperClass;
	}
	public void setMapperClass(Class mapperClass) {
		this.mapperClass = mapperClass; 
	}
	public Map<String, ActionBuilder> getActionsBuilder() {
		return actionsBuilder;
	}
	public void setActionsBuilder(Map<String, ActionBuilder> actionsBuilder) {
		this.actionsBuilder = actionsBuilder;
	}
	public void addActionBuilder(String operationName, ActionBuilder actionBuilder) {
		actionsBuilder.put(operationName, actionBuilder);
	}
	public ActionBuilder getActionBuilder(String operationName) {
		return actionsBuilder.get(operationName);
	}
	public String getServicePrefix() {
		return servicePrefix;
	}
	public void setServicePrefix(String servicePrefix) {
		this.servicePrefix = servicePrefix;
	}
	
	@Override
	public String toString() {
		return "WSIGService (name="+serviceName+", onto="+onto.getName()+", mapper="+mapperClass+")";
	}
}
