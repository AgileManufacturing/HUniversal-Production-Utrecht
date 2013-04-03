package main;

import java.util.ArrayList;
import java.util.List;

import dataClasses.Parameter;
import dataClasses.ProductionStep;
import jade.core.Agent;
import jade.wrapper.AgentController;
import equipletAgent.EquipletAgent;

@SuppressWarnings("serial")
public class MainAgent extends Agent {

	  protected void setup() {
			try {				
				System.out.println("Starting agents:");
				
				//instantiate the 'equiplet' agents
				Object[] ar = new Object[]{ 1 };//pickndplace
				((AgentController)getContainerController().createNewAgent("eqa1", "equipletAgent.EquipletAgent", ar)).start();

				ar = new Object[]{ 2 }; //colour
				((AgentController)getContainerController().createNewAgent("eqa2","equipletAgent.EquipletAgent", ar)).start();
				
				ar = new Object[]{ 3 }; //rotate
				((AgentController)getContainerController().createNewAgent("eqa3","equipletAgent.EquipletAgent", ar)).start();
				
				ar = null;
				
				//Lets make a parameter list
				ArrayList<Parameter> parameterList = new ArrayList<Parameter>();
				parameterList.add(new Parameter("rood", 2, 3));
				parameterList.add(new Parameter("blauw", 2, 3));
				parameterList.add(new Parameter("paars", 2, 3));
				parameterList.add(new Parameter("geel", 2, 3));
				
				//Next we want to have some production steps
				ProductionStep stp1 = new ProductionStep(1, parameterList);
				ProductionStep stp2 = new ProductionStep(2, parameterList);
				ProductionStep stp3 = new ProductionStep(3, parameterList);
				ProductionStep stp4 = new ProductionStep(2, parameterList);
				ProductionStep stp5 = new ProductionStep(1, parameterList);
				
				//Our argument for the product agent. The total production of the product, 
				//consists of multiple steps
				Object[] stepList = new Object[]{
						stp1,
						stp2,
						stp3,
						stp4,
						stp5
				};
				
			((AgentController)getContainerController().createNewAgent("pa1","productAgent.Productagent", stepList)).start();
				
			} catch (Exception e) {
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  } 

}
