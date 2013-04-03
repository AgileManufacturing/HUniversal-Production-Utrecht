package productAgent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

import dataClasses.ProductionStep;
import dataClasses.equipletAgentData;

import jade.core.AID;
import jade.core.Agent;

public class Productagent extends Agent {

public Hashtable<ProductionStep, AID> canPerfStepEquiplet;
public Hashtable canPerfStepWithParamsEquiplet;
public ArrayList<ProductionStep> productionStepList;

	  protected void setup() {
			try {
				
				productionStepList = new ArrayList<ProductionStep>();
				for(Object t : getArguments()){ 
					// At this moment we want to cast each item.
					//This should be updated later however.
					productionStepList.add((ProductionStep)t);	
				}
				
				NegotiatorBehaviour nb = new NegotiatorBehaviour(this);
				addBehaviour(nb);
				//WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
				
				System.out.println("I spawned as a product agent");
				
			} catch (Exception e) {
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  } 
}
