package grid_server;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

public class GridAgent extends Agent{
	private long productAgentCounter =0;
	protected void setup(){	
		addBehaviour(new CyclicBehaviour()
		{ 				
			public void action() {
				ACLMessage msg = receive();
                if (msg!=null) {
                	System.out.println("New Msg");
    				ContainerController cc = getContainerController();
    				String name="ProductAgent-"+productAgentCounter;
					AgentController ac;
					try {
						Object[] arguments = new Object[1];
						arguments[0]=msg.getContent();
						ac = cc.createNewAgent(name, ProductAgent.class.getName(), arguments);
	    				ac.start();
	    				productAgentCounter++;

					} catch (StaleProxyException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
                    System.out.println(msg.getSender().getName()+" Send: "+msg.getContent() );

                    if(!msg.getSender().equals(this.getAgent().getAID())) {  
                    	if(msg.getPerformative()==ACLMessage.INFORM){
                    		
                    	}	                    
                    }
                 }
                block();				
			}		
		});		
	}
	@Override
	protected void takeDown(){
		
	}
	
}
