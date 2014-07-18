package simulation.mas;

import jade.core.Agent;

public class ProductSimAgent extends Agent {

    /**
     * 
     */
    private static final long serialVersionUID = 1L;

    protected void setup() 
    { 
	Object[] args = getArguments();
	String s;
        if (args != null && args.length == 2) {
            
            
            
            for (int i = 0; i<args.length; i++) {
                s = (String) args[i];
                System.out.println("p" + i + ": " + s);
            }
            
            // Extracting the integer.
            int i = Integer.parseInt( (String) args[0] );
            System.out.println("i*i= " + i*i);
            

            System.out.printf("Product agent: %s initialize\n", getLocalName()); 
        } else {
            
            System.out.printf("PA: %s Failed to receive correct arguments\n", getLocalName());
        }
    }
}
