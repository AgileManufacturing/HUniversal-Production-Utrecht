package web_socket_server;

import grid_server.ServerConfigurations;

import java.net.ConnectException;
import java.net.URI;
import java.net.URISyntaxException;

import web_socket_server.java.org.java_websocket.client.WebSocketClient;
import web_socket_server.java.org.java_websocket.handshake.ServerHandshake;

import com.google.gson.JsonObject;

import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.UnreachableException;
import jade.imtp.leap.ICPException;
import jade.wrapper.AgentController;
import jade.wrapper.ControllerException;
import jade.wrapper.StaleProxyException;


public class CreateAgent {	
	/**
	  * @var CONTAINER_NAME
	  * The string holds the container name where in the EquipletAgent is being spawned.
	  */
	private static final String CONTAINER_NAME = "ProductAgentSpawnerAgent";

	
	 /**
	  * main()
	  * Spawns the EquipletAgent on the selected server.
	 * @throws ControllerException 
	  */
	public void createAgent(String args){
		java.util.Date date= new java.util.Date();
		//Spanwing EquipletAgent in the container that has the selected IP/Port
		jade.core.Runtime runtime = jade.core.Runtime.instance();
		
		try {
			Profile profile = new ProfileImpl();		
			profile.setParameter(Profile.MAIN_HOST,ServerConfigurations.GS_IP);
			profile.setParameter(Profile.MAIN_PORT,ServerConfigurations.GS_PORT);
			profile.setParameter(Profile.CONTAINER_NAME,CONTAINER_NAME+date.getTime());
			jade.wrapper.AgentContainer container = runtime.createAgentContainer( profile );
			ProductAgentSpawnerAgent agent = new ProductAgentSpawnerAgent();
			agent.setProductSteps(args);
			try{		
				AgentController ac = container.acceptNewAgent( container.getContainerName(), agent);
				ac.start();		
			}
			catch(ControllerException e){
				AgentController ac;
				try {
					ac = container.acceptNewAgent( container.getContainerName()+"12", agent);
					ac.start();
				} catch (ControllerException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}
		}
		catch(Exception ex){
			System.out.println("create exception");
			try {
				MyWebsocket mws = new MyWebsocket(new URI(ServerConfigurations.WSS_URI));
				boolean connected = mws.connectBlocking();
				System.out.println(connected);
			} catch (URISyntaxException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.out.println("create exception2");
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
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
			send("{\"receiver\":\"interface\", \"message\":\"Could not create product, connection error!!\", \"type\":\"danger\"}");
			close();
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
