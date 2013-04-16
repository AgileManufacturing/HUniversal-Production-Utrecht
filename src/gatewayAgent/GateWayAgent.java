package gatewayAgent;

import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.domain.DFService;
import jade.domain.FIPAException;
import jade.domain.FIPAAgentManagement.DFAgentDescription;
import jade.domain.FIPAAgentManagement.ServiceDescription;
import jade.lang.acl.ACLMessage;
import jade.util.Logger;
import jade.wrapper.AgentController;


public class GateWayAgent extends Agent {

	/**
	 * SerialVersion
	 */
	private static final long serialVersionUID = 1;

	private Logger myLogger = Logger.getMyLogger(getClass().getName());
	
	private void creationOfAgents(){
		String name = "Alice" ;
        jade.wrapper.AgentContainer c = getContainerController();
        try {
            AgentController a = c.createNewAgent( name, "agent.com.Henk", null );
            a.start();
        }
        catch (Exception e){System.out.println(e.getMessage());}
	}
	
	@SuppressWarnings("serial")
	private class WaitPingAndReplyBehaviour extends CyclicBehaviour {

		public WaitPingAndReplyBehaviour(Agent a) {
			super(a);
		}

		public void action() {
			ACLMessage msg = myAgent.receive();
			if(msg != null){
				creationOfAgents();
				
			}
			else {
				block();
			}
		}
	} // END of inner class WaitPingAndReplyBehaviour
	
	@SuppressWarnings("unused")
	private class XMLReader 
	{
		 public void parse(String Message)
		  {
			  
		  }
	}

	protected void setup() {
		// Registration with the DF 
		DFAgentDescription dfd = new DFAgentDescription();
		ServiceDescription sd = new ServiceDescription();   
		sd.setType("SmallTalkAgent"); 
		sd.setName(getName());
		sd.setOwnership("RickyVanRijn");
		dfd.setName(getAID());
		dfd.addServices(sd);
		try {
			DFService.register(this,dfd);
			WaitPingAndReplyBehaviour PingBehaviour = new  WaitPingAndReplyBehaviour(this);
			addBehaviour(PingBehaviour);
		} catch (FIPAException e) {
			myLogger.log(Logger.SEVERE, "Agent "+getLocalName()+" - Cannot register with DF", e);
			doDelete();
		}
	}
}
