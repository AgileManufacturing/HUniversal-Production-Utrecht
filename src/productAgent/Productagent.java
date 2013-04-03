package productAgent;

import java.util.ArrayList;
import java.util.Arrays;

import dataClasses.ProductionStep;
import dataClasses.equipletAgentData;

import jade.core.AID;
import jade.core.Agent;

public class Productagent extends Agent {

public ArrayList<equipletAgentData> canPerfStepEquiplet;
public ArrayList<equipletAgentData> canPerfStepWithParamsEquiplet;
public ArrayList<ProductionStep> productionStepList;

	  protected void setup() {
			try {
				productionStepList = new ArrayList<ProductionStep>(ArrayList(getArguments()));		
				
				NegotiatorBehaviour nb = new NegotiatorBehaviour(this);
				addBehaviour(nb);
				//WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
				
				System.out.println("I spawned as a product agent");
				
			} catch (Exception e) {
				System.out.println("I failed as a product agent");
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  } 
}
