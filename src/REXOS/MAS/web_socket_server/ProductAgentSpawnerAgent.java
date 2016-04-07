package MAS.web_socket_server;

import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;
import jade.wrapper.StaleProxyException;
import util.configuration.ServerConfigurations;


public class ProductAgentSpawnerAgent extends Agent {
	private static final long serialVersionUID = 1L;
	 
	private String productSteps;
	
	public void setProductSteps(String productSteps){
		this.productSteps = productSteps;
	}
	 
	public void sendMessage(String message, int type){
		  ACLMessage acl = new ACLMessage(type);
		  AID aid=new AID(ServerConfigurations.GS_NAME, AID.ISGUID);
		  aid.addAddresses(ServerConfigurations.GS_ADDRESS);
		  acl.addReceiver(aid);
		  acl.setContent(message);
		  System.out.println("PAS: Send acl: " + acl);
		  send(acl);
		  System.out.println("PAS: Send message: " + message);
	}
	protected void setup(){
		System.out.println("PAS: Hello. My name is "+this.getLocalName());
		System.out.println(productSteps);
		System.out.println("PAS: Agent name: "+ this.getName());
		sendMessage(productSteps, ALCMessage.ACCEPT_PROPOSAL);
		try {
			getContainerController().kill();
		} catch (StaleProxyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}