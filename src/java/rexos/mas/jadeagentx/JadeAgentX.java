/**
 * @file JadeAgentX.java
 * @brief Makes the agents with specific variables to test with. 
 * @date Created: 12-04-13
 *
 * @author Wouter Veen
 * 
 * @section LICENSE
 * License: newBSD
 *
 * Copyright � 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

package rexos.mas.jadeagentx;

import jade.core.AID;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;

import rexos.mas.data.Parameter;
import rexos.mas.data.ParameterGroup;
import rexos.mas.data.ParameterList;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionStep;

public class JadeAgentX extends Agent {
	private static final long serialVersionUID = 1L;

	/**
	 * 
	 * the command line arguments
	 */
	@Override
	protected void setup() {
		try {
			System.out.println("starting a agent");

			/**
			 * Make a new logistics agent
			 */
			AgentController logisticsCon = getContainerController()
					.createNewAgent("logistics",
							"rexos.mas.logistics_agent.LogisticsAgent",
							new Object[0]);
			logisticsCon.start();
			AID logisticsAID = new AID(logisticsCon.getName(), AID.ISGUID);

			/**
			 * Make a array list of capabilities
			 */
			ArrayList<Integer> capabilities1 = new ArrayList<>();
			capabilities1.add(1);
			capabilities1.add(2);
			capabilities1.add(3);

			/**
			 * make a new equipletagent to use.
			 */
			Object[] ar = new Object[] { capabilities1, logisticsAID };
			getContainerController().createNewAgent("EQ1",
					"rexos.mas.equiplet_agent.EquipletAgent", ar).start();
			// TODO code application logic here

			ArrayList<Integer> capabilities2 = new ArrayList<>();
			capabilities2.add(1);
			capabilities2.add(2);
			ar = new Object[] { capabilities2, logisticsAID };
			getContainerController().createNewAgent("EQ2",
					"rexos.mas.equiplet_agent.EquipletAgent", ar).start();

			ArrayList<Integer> capabilities3 = new ArrayList<>();
			capabilities3.add(1);
			capabilities3.add(2);
			ar = new Object[] { capabilities3, logisticsAID };
			getContainerController().createNewAgent("EQ3",
					"rexos.mas.equiplet_agent.EquipletAgent", ar).start();

			ar = null;

			/**
			 * Lets make a parameter list
			 */
			ParameterList parameterList = new ParameterList();
			ParameterGroup p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "1"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "2"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);

			// Next we want to have some production steps
			ProductionStep stp1 = new ProductionStep(1, 1, parameterList);

			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "3"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "4"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);

			ProductionStep stp2 = new ProductionStep(2, 1, parameterList);

			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "5"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "6"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);

			ProductionStep stp3 = new ProductionStep(3, 2, parameterList);

			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "7"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "8"));
			parameterList.AddParameterGroup(p);

			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);

			ProductionStep stp4 = new ProductionStep(4, 3, parameterList);

			/**
			 * Our argument for the product agent. The total production of the
			 * product, consists of multiple steps
			 */
			ArrayList<ProductionStep> stepList = new ArrayList<>();
			stepList.add(stp1);
			stepList.add(stp2);
			stepList.add(stp3);
			stepList.add(stp4);

			Production production = new Production(stepList);
			Product product = new Product(production, getAID().toString());

			/**
			 * We need to pass an Object[] to the createNewAgent. But we only
			 * want to pass our product!
			 */

			Object[] args = new Object[1];
			args[0] = product;

			getContainerController().createNewAgent("pa" + count++,
					"rexos.mas.productAgent.ProductAgent", args).start();
			addBehaviour(new StartProductAgent(this, args));
		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}
	}

	static int count = 0;

	public class StartProductAgent extends CyclicBehaviour {
		private static final long serialVersionUID = 1L;

		Object[] args;

		/**
		 * 
		 * @param a
		 * @param args
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
			if (message != null) {
				try {
					getContainerController().createNewAgent("pa" + count++,
							"rexos.mas.productAgent.ProductAgent", args)
							.start();
				} catch (StaleProxyException e) {
					e.printStackTrace();
				}
			}
			block();
		}
	}
}
