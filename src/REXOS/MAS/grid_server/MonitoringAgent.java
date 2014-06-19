package grid_server;

import grid_server.ServerConfigurations;

import java.net.URI;
import java.net.URISyntaxException;

import web_socket_server.java.org.java_websocket.client.WebSocketClient;
import web_socket_server.java.org.java_websocket.handshake.ServerHandshake;
import jade.core.Agent;
import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import jade.wrapper.AgentController;
import jade.wrapper.ContainerController;
import jade.wrapper.StaleProxyException;

public class MonitoringAgent extends Agent{
	MyWebsocket mws;

	protected void setup(){	
		try {
			 mws= new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
			boolean connected = mws.connectBlocking();
		} catch (URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println("create exception2");
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		addBehaviour(new HeartBeatBehaviour(this, 10000));
		addBehaviour(new CyclicBehaviour()
		{ 				
			public void action() {
				ACLMessage msg = receive();
                if (msg!=null) {
                	mws.send(msg.getContent());
                 }
                block();				
			}		
		});		
	}
	@Override
	protected void takeDown(){
		
	}
	


	private class MyWebsocket extends WebSocketClient {
		public MyWebsocket(URI serverURI) {
			super(serverURI);
			// TODO Auto-generated constructor stub
		}
	
		@Override
		public String getResourceDescriptor() {
			// TODO Auto-generated method stub
			return null;
		}
	
		@Override
		public void onOpen(ServerHandshake handshakedata) {
			// TODO Auto-generated method stub
			System.out.println("onOpen");
			//send("{\"receiver\":\"interface\", \"message\":\"Could not create product, connection error!!\", \"type\":\"danger\"}");
			//close();
		}
	
		@Override
		public void onMessage(String message) {
			// TODO Auto-generated method stub
			
		}
	
		@Override
		public void onClose(int code, String reason, boolean remote) {
			// TODO Auto-generated method stub
			System.out.println("onClose");
			
		}
	
		@Override
		public void onError(Exception ex) {
			// TODO Auto-generated method stub
			
		}		
	}
}
