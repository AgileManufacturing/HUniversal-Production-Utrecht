/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jadeagentx;

import equipletAgent.EquipletAgent;
import serviceAgent.ServiceAgent;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.core.behaviours.OneShotBehaviour;
import jade.core.behaviours.SimpleBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

import newDataClasses.Parameter;
import newDataClasses.ParameterGroup;
import newDataClasses.ParameterList;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionStep;

/**
 * 
 * @author wouter
 */
public class JadeAgentX extends Agent {
	private static final long serialVersionUID = 1L;

	/**
	 * @param args
	 *            the command line arguments
	 */
	protected void setup() {
		try {
			System.out.println("starting a agent");

			ArrayList<Long> capabilities1 = new ArrayList<Long>();
			capabilities1.add(1l);
			capabilities1.add(11l);
			capabilities1.add(24l);
			capabilities1.add(23412l);
			capabilities1.add(15l);

			Object[] ar = new Object[] { capabilities1 };
			((AgentController) getContainerController().createNewAgent("eqa1", "equipletAgent.EquipletAgent", ar)).start();
			// TODO code application logic here
			ArrayList<Long> capabilities2 = new ArrayList<Long>();
			capabilities2.add(2l);
			capabilities2.add(5l);
			capabilities2.add(9l);

			ar = new Object[] { capabilities2 };
			((AgentController) getContainerController().createNewAgent("eqa2", "equipletAgent.EquipletAgent", ar)).start();

			ArrayList<Long> capabilities3 = new ArrayList<Long>();
			capabilities3.add(3l);
			capabilities3.add(4l);
			capabilities3.add(7l);
			capabilities3.add(9l);

			ar = new Object[] { capabilities3 };
			((AgentController) getContainerController().createNewAgent("eqa3", "equipletAgent.EquipletAgent", ar)).start();

			ar = null;

			// Lets make a parameter list
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
			ProductionStep stp1 = new ProductionStep(1, parameterList);

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

			ProductionStep stp2 = new ProductionStep(2, parameterList);

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

			ProductionStep stp3 = new ProductionStep(3, parameterList);

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

			ProductionStep stp4 = new ProductionStep(4, parameterList);

			// Our argument for the product agent. The total production of the
			// product,
			// consists of multiple steps
			ProductionStep[] stepList = new ProductionStep[] { stp1, stp2, stp3, stp4 };

			Production production = new Production(stepList);
			Product product = new Product(production);

			// We need to pass an Object[] to the createNewAgent.
			// But we only want to pass our product!
			Object[] args = new Object[1];
			args[0] = product;

			addBehaviour(new StartProductAgent(this, args));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static long count = 0;

	public class StartProductAgent extends CyclicBehaviour {
		private static final long serialVersionUID = 1L;

		Object[] args;

		public StartProductAgent(Agent a, Object[] args) {
			super(a);
			this.args = args;
		}

		@Override
		public void action() {
			ACLMessage message = receive();
			if (message != null) {
				try {
					((AgentController) getContainerController().createNewAgent("pa" + count++, "productAgent.ProductAgent", args)).start();
				} catch (StaleProxyException e) {
					e.printStackTrace();
				}
			}
		}
	}
}