
package main;

import jade.core.Agent;
import jade.wrapper.AgentController;
import newDataClasses.Parameter;
import newDataClasses.ParameterGroup;
import newDataClasses.ParameterList;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionStep;

/*
 * Author: Alexander
 * 
 * Dummy main agent for testing purposes.
 * Starts up the different agents and sets the (for now) hardcoded information.
 * 
 * DISCLAIMER
 * I DID NOT FOLLOW ANY CODING REGULATIONS. THIS IS NOT OFFICIAL PROJECT CODE,
 * NOR WILL IT BE USED FOR ANYTHING OTHER THEN TESTING.
 * COMMENTS ARE SCARCE AND CRAPPY
 *  
 *  USE AT OWN RISK.
 */
@SuppressWarnings("serial")
public class MainAgent extends Agent {
	protected void setup() {
		try {
			System.out.println("Starting agents:");

			// instantiate the 'equiplet' agents
			Object[] ar = new Object[] { 1 };// pickndplace
			((AgentController) getContainerController().createNewAgent("eqa1",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 2 }; // colour
			((AgentController) getContainerController().createNewAgent("eqa2",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 3 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa3",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 2 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa4",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 1 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa5",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 3 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa6",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 1 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa7",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 1 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa8",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 2 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa9",
					"equipletAgent.EquipletAgent", ar)).start();
			ar = new Object[] { 3 }; // rotate
			((AgentController) getContainerController().createNewAgent("eqa10",
					"equipletAgent.EquipletAgent", ar)).start();

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
			ProductionStep stp1 = new ProductionStep(1, 0, parameterList);

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

			// Our argument for the product agent. The total production of the
			// product,
			// consists of multiple steps
			ProductionStep[] stepList = new ProductionStep[] { stp1, stp2,
					stp3, stp4 };

			Production production = new Production(stepList);
			Product product = new Product(production);

			//We need to pass an Object[] to the createNewAgent. 
			// But we only want to pass our product!
			Object[] arg = new Object[1];
			arg[0] = product;

			((AgentController) getContainerController().createNewAgent("pa1",
					"productAgent.ProductAgent", arg)).start();

		} catch (Exception e) {
			e.printStackTrace();
			doDelete();
		}
	}
}
