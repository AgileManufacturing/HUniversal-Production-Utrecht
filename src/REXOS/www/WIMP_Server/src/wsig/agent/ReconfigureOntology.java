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

package wsig.agent;

import jade.content.onto.BasicOntology;
import jade.content.onto.Ontology;
import jade.content.onto.OntologyException;
import jade.content.schema.AgentActionSchema;
import jade.content.schema.ConceptSchema;
import jade.content.schema.ObjectSchema;
import jade.content.schema.PrimitiveSchema;
import jade.content.schema.TermSchema;
import agents.equiplet_agent.reconfigure.behaviours.ReconfigureBehaviour;

public class ReconfigureOntology extends Ontology implements ReconfigureVocabulary{
	private static final long serialVersionUID = 1L;
	private final static Ontology theInstance = new ReconfigureOntology();

	public final static Ontology getInstance() {
		return theInstance;
	}

	/**
	 * Constructor
	 */
	public ReconfigureOntology() {
		super(ONTOLOGY_NAME, BasicOntology.getInstance());

		try {

			add(new ConceptSchema(AGENTINFO), AgentInfo.class);
			
			add(new AgentActionSchema(RECONFIGURE), Class.forName("wsig.agent.ReconfigureOntology"));
			add(new AgentActionSchema(GETMODULES), Class.forName("wsig.agent.ReconfigureOntology"));
			//add(new AgentActionSchema(GETMODULES), ReconfigureBehaviour.class);
			
//			AgentActionSchema as = (AgentActionSchema) getSchema(GETMODULES);
//			as.add(JSON, (PrimitiveSchema) getSchema(BasicOntology.STRING));
//			as.setResult((PrimitiveSchema) getSchema(BasicOntology.STRING), 0, 
//					ObjectSchema.UNLIMITED);
//
//			
//			as = (AgentActionSchema) getSchema(RECONFIGURE);
//			as.add(JSON, (PrimitiveSchema) getSchema(BasicOntology.STRING));
//			as.setResult((PrimitiveSchema) getSchema(BasicOntology.STRING));
		
		} catch (OntologyException oe) {
			oe.printStackTrace();
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
