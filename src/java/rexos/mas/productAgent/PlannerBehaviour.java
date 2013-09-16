/**
 * @file rexos/mas/productAgent/PlannerBehaviour.java
 * @brief Behaviour in which the product agent reads from the equiplet directory
 *        blackboard to see which equiplets are capable to perform the step.
 * @date Created: 02-04-2013
 * 
 * @author Arno Derks
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
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

package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.Behaviour;
import rexos.mas.data.BehaviourStatus;
import rexos.mas.data.LogLevel;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.StepStatusCode;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.log.Logger;

import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

import configuration.Configuration;
import configuration.ConfigurationFiles;

public class PlannerBehaviour extends Behaviour {

	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;
	private BehaviourCallback _bc;

	private boolean _isDone = false;
	private boolean _isError = false;
	private boolean _isCompleted = false;

	/**
	 * Constructs the planne behavior
	 * 
	 * @param myAgent
	 * @param bc
	 */
	public PlannerBehaviour(Agent myAgent, BehaviourCallback bc) {
		super(myAgent);
		this._bc = bc;
	}

	/**
	 * Starts the planning
	 */
	@Override
	public void onStart() {
		try {
			_productAgent = (ProductAgent) myAgent;

			BlackboardClient bbc = new BlackboardClient(
					Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"), 
					Integer.parseInt(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbPort")));
			
			bbc.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbName"));
			bbc.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletDirectoryName"));
			
			
			Product product = this._productAgent.getProduct();
			Production production = product.getProduction();
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			ProductionEquipletMapper prodEQmap = production
					.getProductionEquipletMapping();

			// Iterate over all the production steps
			for (ProductionStep prodStep : psa) {
				if (prodStep.getStatus() == StepStatusCode.EVALUATING
						|| prodStep.getStatus() == StepStatusCode.RESCHEDULE) {

					int PA_id = prodStep.getId();
					prodEQmap.addProductionStep(PA_id);
					// Get the type of production step, aka capability
					long PA_capability = prodStep.getCapability();
					// Create the select query for the blackboard
					DBObject equipletCapabilityQuery = QueryBuilder
							.start("capabilities").is(PA_capability).get();
					List<DBObject> equipletDirectory = bbc
							.findDocuments(equipletCapabilityQuery);
					for (DBObject DBobj : equipletDirectory) {
						DBObject aid = (DBObject) DBobj.get("db");
						String name = aid.get("name").toString();
						prodEQmap.addEquipletToProductionStep(PA_id, new AID(
								name, AID.ISLOCALNAME));
					}
				} else {
				}
			}

			production.setProductionEquipletMapping(prodEQmap);
			product.setProduction(production);
			this._productAgent.setProduct(product);
			this._isDone = true;
		} catch (NullPointerException | InvalidDBNamespaceException
				| GeneralMongoException | UnknownHostException e) {
			Logger.log(LogLevel.ERROR,
					"Exception PlannerBehaviour - onStart() : ", e);
			this._isError = true;
		}
	}

	/**
	 * Constructs the status of the planner behavior
	 */
	@Override
	public void action() {

		if (this._isDone) {
			this._bc.handleCallback(BehaviourStatus.COMPLETED, null);
			this._isCompleted = true;
		} else if (this._isError) {
			this._bc.handleCallback(BehaviourStatus.ERROR, null);
			this._isCompleted = true;
		} else {
			block();
		}
	}

	@Override
	public void reset() {
		super.reset();
		_isDone = false;
		_isError = false;
		_isCompleted = false;
	}

	/**
	 * returns true when the behavior is done
	 * 
	 * @return
	 */
	@Override
	public boolean done() {
		return this._isCompleted;
	}
}
