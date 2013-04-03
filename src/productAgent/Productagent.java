package productAgent;

import jade.core.Agent;

public class Productagent extends Agent {

	  protected void setup() {
			try {
				
				
				//WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
				//addBehaviour(behaviour);
				
				System.out.println("I spawned as a product agent");
				
			} catch (Exception e) {
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  } 
}
