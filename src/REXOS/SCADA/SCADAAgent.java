package SCADA;

import java.net.UnknownHostException;

public class SCADAAgent {

	private SCADAWebSocketServer webSocketServer;
	private static final int webSocketPort = 3529;
	
	public SCADAAgent() {
		try {
			webSocketServer = new SCADAWebSocketServer(webSocketPort);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
}
