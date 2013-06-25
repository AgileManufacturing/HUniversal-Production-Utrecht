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

package rexos.mas.jadeagentx;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.log.Logger;
import rexos.mas.data.Callback;
import rexos.mas.data.Position;
import rexos.mas.data.Product;
import rexos.mas.data.ProductAgentProperties;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;

import com.mongodb.BasicDBObject;

/**
 *	Test class for testing the equiplet agent, service agent and hardware agent.
 */
public class JadeAgentX extends Agent {
	private static final long serialVersionUID = 1L;

	/**
	 * 
	 * the command line arguments
	 */
	@Override
	protected void setup() {
		try {
			Logger.log("starting an agent");

			/**
			 * Make a new logistics agent
			 */
			AgentController logisticsCon =
					getContainerController().createNewAgent("logistics", "rexos.mas.logistics_agent.LogisticsAgent",
							new Object[0]);
			logisticsCon.start();
			AID logisticsAID = new AID(logisticsCon.getName(), AID.ISGUID);

			// Empty the equiplet directory before starting the first equiplet agent
			BlackboardClient collectiveBBClient = new BlackboardClient("145.89.191.131", 27017);
			collectiveBBClient.setDatabase("CollectiveDb");
			collectiveBBClient.setCollection("EquipletDirectory");
			collectiveBBClient.removeDocuments(new BasicDBObject());

			/**
			 * make a new equipletagent to use.
			 */
			Object[] ar = new Object[] {
				logisticsAID
			};
			getContainerController().createNewAgent("EQ1", "rexos.mas.equiplet_agent.EquipletAgent", ar).start();

			/**
			 * Lets make a parameter list
			 */
			BasicDBObject parameters = new BasicDBObject();
//			parameters.append("part", new Part(1).toBasicDBObject());
//			parameters.append("position", new Position(1.0, 2.0, 3.0, new Part(2)).toBasicDBObject());
			parameters.append("startPosition", new Position(1.0, 1.0).toBasicDBObject());
			parameters.append("endPosition", new Position(1.0, 1.0).toBasicDBObject());

			// Next we want to have some production steps
			ProductionStep stp1 = new ProductionStep(1, 3, parameters);
//			ProductionStep stp2 = new ProductionStep(2, 1, parameters);
//			ProductionStep stp3 = new ProductionStep(3, 1, parameters);
//			ProductionStep stp4 = new ProductionStep(4, 1, parameters);

			/**
			 * Our argument for the product agent. The total production of the
			 * product, consists of multiple steps
			 */
			ArrayList<ProductionStep> stepList = new ArrayList<>();
			stepList.add(stp1);
			// stepList.add(stp2);
			// stepList.add(stp3);
			// stepList.add(stp4);

			Production production = new Production(stepList);
			Product product = new Product(production);
			
			Callback callback = new Callback();
			callback.setHost("127.0.0.1");
			callback.setPort(1233);
			
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

			getContainerController().createNewAgent("pa" + count++, "rexos.mas.productAgent.ProductAgent", args)
					.start();
			addBehaviour(new StartProductAgent(this, args));
		} catch(Exception e) {
			Logger.log(e);
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
		 * 		The agent this behaviour is linked to/this test agent.
		 * @param args
		 * 		The arguments for the product agent.
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
							.createNewAgent("pa" + count++, "rexos.mas.productAgent.ProductAgent", args).start();
				} catch(StaleProxyException e) {
					Logger.log(e);
				}
			}
			block();
		}
	}
}
