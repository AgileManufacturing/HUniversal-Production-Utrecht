package productAgent;

import java.util.ArrayList;

import dataClasses.equipletAgentData;

import jade.core.AID;
import jade.core.Agent;

public class Productagent extends Agent {

public ArrayList<equipletAgentData> canPerfStepEquiplet;
public ArrayList<equipletAgentData> canPerfStepWithParamsEquiplet;
	
	  protected void setup() {
			try {
				Object[] stepList =	null;			
				
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
