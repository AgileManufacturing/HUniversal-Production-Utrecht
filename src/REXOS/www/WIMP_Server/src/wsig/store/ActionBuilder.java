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

import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import org.apache.log4j.Logger;

import jade.content.AgentAction;
import jade.content.onto.Ontology;
import jade.content.onto.OntologyException;
import jade.content.schema.AgentActionSchema;
import jade.content.schema.ObjectSchema;

public abstract class ActionBuilder {
	
	protected static Logger log = Logger.getLogger(OntologyBasedActionBuilder.class.getName());

	private Ontology onto;
	private String ontoActionName;
	private Map<String, ObjectSchema> parametersMap = new HashMap<String, ObjectSchema>();

	public abstract AgentAction getAgentAction(Vector<ParameterInfo> params) throws Exception;

	public ActionBuilder(Ontology onto, String ontoActionName) {
		this.onto = onto;
		this.ontoActionName = ontoActionName;
	}

	public Ontology getOntology() {
		return onto;
	}

	public String getOntologyActionName() {
		return ontoActionName;
	}
	
	public Map<String, ObjectSchema> getParametersMap() {
		return parametersMap;
	}

	public void setParametersMap(Map<String, ObjectSchema> parametersMap) {
		this.parametersMap = parametersMap;
	}

	public AgentActionSchema getOntologyActionSchema() throws OntologyException {
		return (AgentActionSchema) onto.getSchema(ontoActionName);
	}
}
