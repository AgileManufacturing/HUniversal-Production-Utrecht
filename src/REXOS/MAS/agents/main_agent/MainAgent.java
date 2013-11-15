/**
 * @file rexos/mas/jadeagentx/JadeAgentX.java
 * @brief Makes the agents with specific variables to test with.
 * @date Created: 12-04-13
 * 
 * @author Wouter Veen
 * 
 * @section LICENSE
 *          License: newBSD
 * 
 *          Copyright � 2013, HU University of Applied Sciences Utrecht.
 *          All rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met:
 *          - Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *          - Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *          - Neither the name of the HU University of Applied Sciences Utrecht
 *          nor the names of its contributors may be used to endorse or promote
 *          products derived from this software without specific prior written
 *          permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO,
 *          THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *          PARTICULAR PURPOSE
 *          ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED
 *          SCIENCES UTRECHT
 *          BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 *          OR
 *          CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *          SUBSTITUTE
 *          GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *          INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT
 *          LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *          ANY WAY OUT
 *          OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *          SUCH DAMAGE.
 **/

package agents.main_agent;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;

import libraries.blackboard_client.BlackboardClient;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import agents.data_classes.Callback;
import agents.data_classes.Product;
import agents.data_classes.ProductAgentProperties;
import agents.data_classes.Production;
import agents.data_classes.ProductionStep;
import agents.logistics_agent.behaviours.ArePartsAvailable;

import com.mongodb.BasicDBObject;

import configuration.Configuration;
import configuration.ConfigurationFiles;

/**
 * Test class for testing the equiplet agent, service agent and hardware agent.
 */
public class MainAgent extends Agent {
	private static final long serialVersionUID = 1L;

	/**
	 * 
	 * the command line arguments
	 */
	@Override
	protected void setup() {
		try {
			Logger.log(LogLevel.DEBUG, "starting the main agent");

			/**
			 * Make a new logistics agent
			 */
			AgentController logisticsCon =
					getContainerController().createNewAgent("Logistics", "agents.logistics_agent.LogisticsAgent",
							new Object[0]);
			
			logisticsCon.start();
			AID logisticsAID = new AID(logisticsCon.getName(), AID.ISGUID);

			// Empty the equiplet directory before starting the first equiplet agent
			BlackboardClient collectiveBBClient = new BlackboardClient(
					Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbIp"), 
					Configuration.getPropertyInt(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbPort"));
					
			collectiveBBClient.setDatabase(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "collectiveDbName"));
			collectiveBBClient.setCollection(Configuration.getProperty(ConfigurationFiles.MONGO_DB_PROPERTIES, "equipletDirectoryName"));
			collectiveBBClient.removeDocuments(new BasicDBObject());

			/**
			 * make a new equipletagent to use.
			 */
			Object[] ar = new Object[] {
				logisticsAID
			};
			
			getContainerController().createNewAgent("EQ1", "agents.equiplet_agent.EquipletAgent", ar).start();

			Logger.log(LogLevel.DEBUG, "Started equiplet agent");

			// Create parameters
			BasicDBObject placeParameters1 = new BasicDBObject();
			placeParameters1.append("part", 1); // 1: Red Ball
			placeParameters1.append("row", 0); // First row in a crate
			placeParameters1.append("column", 0); // First column in a crate
			
			BasicDBObject placeParameters2 = new BasicDBObject();
			placeParameters2.append("part", 1); // 1: Red Ball
			placeParameters2.append("row", 3); // Fourth row in a crate
			placeParameters2.append("column", 3); // Fourth column in a crate
			
			// Create steps
			ProductionStep place1 = new ProductionStep(1, 1, placeParameters1);
			ProductionStep place2 = new ProductionStep(2, 1, placeParameters2);
			
			// Add them to stepList
			ArrayList<ProductionStep> stepList = new ArrayList<>();
			stepList.add(place1);
			stepList.add(place2);

			Production production = new Production(stepList);
			Product product = new Product(production);

			Callback callback = new Callback();
			callback.setHost("145.89.84.156");
			callback.setPort(21);

			ProductAgentProperties pap = new ProductAgentProperties();
			pap.setCallback(callback);
			pap.setProduct(product);

			/**
			 * We need to pass an Object[] to the createNewAgent. But we only
			 * want to pass our product!
			 */

			Thread.sleep(1000);
			Object[] args = new Object[1];
			args[0] = pap;
			
			addBehaviour(new StartProductAgent(this, args));
		} catch(Exception e) {
			e.printStackTrace();
			doDelete();
		}
	}

	static int count = 0;

	/**
	 * Behaviour for starting a product agent.
	 * 
	 */
	public class StartProductAgent extends CyclicBehaviour {
		private static final long serialVersionUID = 1L;

		Object[] args;

		/**
		 * Constructor for the StartProductAgent behaviour.
		 * 
		 * @param a
		 *            The agent this behaviour is linked to/this test agent.
		 * @param args
		 *            The arguments for the product agent.
		 */
		public StartProductAgent(Agent a, Object[] args) {
			super(a);
			this.args = args;
		}

		/**
		 * Make new product agent
		 */
		@Override
		public void action() {
			ACLMessage message = receive();
			if(message != null) {
				try {
					getContainerController()
							.createNewAgent("pa" + count++, "agents.product_agent.ProductAgent", args).start();
				} catch(StaleProxyException e) {
					Logger.log(LogLevel.ERROR, "", e);
				}
			}
			block();
		}
	}
}

