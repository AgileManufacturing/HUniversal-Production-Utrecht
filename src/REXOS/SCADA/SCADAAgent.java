package SCADA;

import jade.core.Agent;

import java.net.UnknownHostException;

public class SCADAAgent extends Agent implements WebSocketServerListener{

	/**
	 *  Default
	 */
	private static final long serialVersionUID = 1L;
	
	private SCADAWebSocketServer webSocketServer;
	private static final int webSocketServerPort = 3529;
	
	protected void setup() {
		
		try {
			webSocketServer = new SCADAWebSocketServer(webSocketServerPort);
			webSocketServer.start();
			webSocketServer.setListener(this);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void onWebSocketMessage(String message) {
		//convertToAction?
		System.out.println("SCADAAgent received: " + message);
	}
	
	
	
	
	
}
