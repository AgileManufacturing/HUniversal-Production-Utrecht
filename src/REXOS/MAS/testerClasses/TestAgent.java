package MAS.testerClasses;


import java.net.URI;
import java.net.URISyntaxException;

import MAS.util.Ontology;
import MAS.web_socket_server.ProductAgentSpawnerAgent;
import jade.core.AID;
import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.ControllerException;
import org.java_websocket.client.*;
import org.java_websocket.handshake.ServerHandshake;

import util.configuration.ServerConfigurations;
public class TestAgent extends Agent {

	private static final long serialVersionUID = 1L;
	int amountOfMessages = 0;
	int amountOfMessagesSocket = 0;
	
	protected void setup() {
		addBehaviour(new TestAgentListenerBehaviour(this));
		System.out.println("Agent: " + getLocalName() + " started");
	}
	
	public void sendMessage(){
		amountOfMessages++;
		// get agent.
		// TestReceiverAgent
		String name = "TRA";
		AID agent = new AID(name, AID.ISLOCALNAME);
		//send message.
		String JSONMessage = "{\n 'requested-equiplet-command': 'TESTMESSAGE',\n 'name': '" + getLocalName() + "', \n 'id':" + amountOfMessages +",\n 'time':" + System.currentTimeMillis() + "\n }";
		ACLMessage message = new ACLMessage(ACLMessage.PROPOSE);
		message.addReceiver(agent);
		message.setOntology(Ontology.GRID_ONTOLOGY);
		message.setConversationId(Ontology.CONVERSATION_EQUIPLET_COMMAND);
		message.setContent(JSONMessage);
		send(message);
		System.out.println("send");
	}
	
	public void sendMessageSocket(){
		amountOfMessagesSocket++;
		String JSONMessage = "{\n 'requested-equiplet-command': 'TESTMESSAGE',\n 'name': '" + getLocalName() + "', \n 'id':" + amountOfMessagesSocket +",\n 'time':" + System.currentTimeMillis() + "\n }";
		try {
			MyWebsocket mws = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
			mws.setCreated(true);
			mws.connect();
			boolean send = false;
			while(!send){
				if(mws.isOpen()){
					mws.send(JSONMessage);
					mws.close();
					send = true;
				}
			}
		} catch (Exception ex) {
			try {
				MyWebsocket mws = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
				mws.setCreated(false);
				mws.connectBlocking();
				boolean send = false;
				while(!send){
					if(mws.isOpen()){
						mws.send(JSONMessage);
						mws.close();
						send = true;
					}
				}
			} catch (URISyntaxException e) {
				e.printStackTrace();
				System.out.println("Impossible, URL format incorrect: " + ServerConfigurations.WSS_URI);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
	}

	private class MyWebsocket extends WebSocketClient {
		boolean created = false;

		public void setCreated(boolean created) {
			this.created = created;
		}

		public MyWebsocket(URI serverURI) {
			super(serverURI);
		}

		@Override
		public String getResourceDescriptor() {
			return null;
		}

		@Override
		public void onOpen(ServerHandshake handshakedata) {
			// TODO Auto-generated method stub
			System.out.println("onOpen");
			if (created) {
				send("{\"receiver\":\"interface\", \"message\":\"Created product\", \"type\":\"success\"}");
			} else {
				send("{\"receiver\":\"interface\", \"message\":\"Could not create product, connection error!!\", \"type\":\"danger\"}");
			}

			close();
		}

		@Override
		public void onMessage(String message) {
		}

		@Override
		public void onClose(int code, String reason, boolean remote) {
		}

		@Override
		public void onError(Exception ex) {
		}
	}
}

