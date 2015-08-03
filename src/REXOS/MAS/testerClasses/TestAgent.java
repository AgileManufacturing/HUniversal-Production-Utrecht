package MAS.testerClasses;


import java.net.URI;
import java.net.URISyntaxException;

import MAS.util.Ontology;
import jade.core.AID;
import jade.core.Agent;
import jade.lang.acl.ACLMessage;

import org.java_websocket.client.*;
import org.java_websocket.handshake.ServerHandshake;

public class TestAgent extends Agent {

	private static final long serialVersionUID = 1L;
	int amountOfMessages = 0;
	int amountOfMessagesSocket = 0;
	MyWebsocket mws;
	
	
	protected void setup() {
		addBehaviour(new TestAgentListenerBehaviour(this));
		try {
			mws = new MyWebsocket(new URI("ws://127.0.0.1:5000"));
			mws.connect();
		}catch(URISyntaxException e) {
				// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
		mws.send(JSONMessage);
	}

	private class MyWebsocket extends WebSocketClient {
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
			System.out.println("Open");
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

