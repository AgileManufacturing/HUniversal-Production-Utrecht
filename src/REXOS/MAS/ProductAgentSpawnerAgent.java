
import jade.core.AID;
import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;

import agents.equiplet_agent.EquipletAgent;
import agents.product_agent.ProductAgent;

public class ProductAgentSpawnerAgent extends Agent {
	 private static final String SERVER_NAME = "Grid@Platform2";
	 private static final String SERVER_ADDRESS = "http://Tommy-PC.wired.hu.nl:7778/acc";
	 
	public void sendMessage(String message, int type){
		  ACLMessage acl = new ACLMessage(type);
		  AID aid=new AID(SERVER_NAME,AID.ISGUID);
		  aid.addAddresses(SERVER_ADDRESS);
		  
		  acl.addReceiver(aid);
		  acl.setContent(message);
		  send(acl);
		  System.out.println("Send message: " + message);
		 }
	protected void setup(){
		Object[] arguments =this.getArguments();
		String arg="";
		for(int i = 0; i < arguments.length; i++){
			arg += arguments[i].toString();
		}
		sendMessage(arg, 0);
		try {
			getContainerController().kill();
		} catch (StaleProxyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}