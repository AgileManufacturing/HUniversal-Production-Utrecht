package productAgent;

import jade.core.Agent;
import newDataClasses.Product;

public class Productagent extends Agent {
private Product _product;

	  protected void setup() {
			try {
				_product = (Product) getArguments()[0];
				
				NegotiatorBehaviour nb = new NegotiatorBehaviour(this);
				addBehaviour(nb);
				//WaitMsgBehaviour behaviour = new WaitMsgBehaviour();
				
				System.out.println("I spawned as a product agent");
				
			} catch (Exception e) {
				System.out.println("Exited with: " + e);
				doDelete();
			}
	  }
	  
	  public Product getProduct(){
		  return this._product;
	  }
}
