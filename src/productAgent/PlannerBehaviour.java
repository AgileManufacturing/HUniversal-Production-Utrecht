/**
 * @file PlannerBehaviour.java
 * @brief Behaviour in which the product agent reads from the equiplet directory
 *        blackboard to see which equiplets are capable to perform the step.
 * @date Created: 02-04-2013
 * 
 * @author Arno Derks
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

package productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;

import java.util.ArrayList;
import java.util.List;

import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import libraries.blackboardJavaClient.src.nl.hu.client.InvalidDBNamespaceException;
import libraries.blackboardJavaClient.src.nl.hu.client.InvalidJSONException;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;
import newDataClasses.ProductionStepStatus;

import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

public class PlannerBehaviour extends OneShotBehaviour{
	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;

	public void plannerBehaviour(){
	}

	@Override
	public int onEnd(){
		return 0;
	}

	public static void removeEquiplet(AID aid){
		BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
		// try to remove the given 'aid' from the blackboard (for testing
		// purposes only)
		try{
			bbc.removeDocuments(aid.toString());
		} catch(InvalidJSONException | InvalidDBNamespaceException e){
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void action(){
		try{
			// Get the root Agent
			_productAgent = (ProductAgent) myAgent;
			// Create the blackboardclient to connect to a specific IP & port
			BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
			// Select the database
			bbc.setDatabase("CollectiveDb");
			// Select the collection
			bbc.setCollection("EquipletDirectory");
			// Get the product object
			Product product = this._productAgent.getProduct();
			// Get the production object
			Production production = product.getProduction();
			// Retrieve the productionstep array
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			// Retrieve the equipletmapper
			ProductionEquipletMapper pem = production
					.getProductionEquipletMapping();
			// Iterate over all the production steps
			for(ProductionStep ps : psa){
				if (ps.getStatus() == ProductionStepStatus.STATE_TODO){
					// Get the ID for the production step
					int PA_id = ps.getId();
					// Get the type of production step, aka capability
					long PA_capability = ps.getCapability();
					// Create the select query for the blackboard
					DBObject equipletCapabilityQuery = QueryBuilder
							.start("capabilities").is(PA_capability).get();
					List<DBObject> equipletDirectory = bbc
							.findDocuments(equipletCapabilityQuery);
					for(DBObject dbo : equipletDirectory){
						DBObject aid = (DBObject) dbo.get("db");
						String name = aid.get("name").toString();
						pem.addEquipletToProductionStep(PA_id, new AID(name,
								AID.ISLOCALNAME));
					}
				}
			}
			// Set the production mapper in the production object
			production.setProductionEquipletMapping(pem);
			// Add the production to the product object
			product.setProduction(production);
			// Set the product object in the product agent
			this._productAgent.setProduct(product);
		} catch(Exception e){
			System.out.println("Exception planner " + e);
		}
	}
}
