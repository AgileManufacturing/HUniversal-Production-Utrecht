package gatewayserver;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

/**
 *
 * Project: GatewayServer
 *
 * Package: 
 *
 * File: main.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */

/**
 * @author Mike
 * 
 */
public class GatewayServer implements Runnable {

	private int _serverSocketPort;

	private ServerSocket _serverSocket;


	private boolean _stopServer = false;

	private int productAgentID = 10;
	
	private String agentHost = "";
	
	private int cooldown = 0;
	
	private long lastTimeOfCooldown = 0;
	
	private final int cooldownLength = 10000;
	
	public GatewayServer(int port, String agentHost) throws IOException {
		super();
		this.agentHost = agentHost;
		this._serverSocketPort = port;
	}

	public void run() {
		try {
			Socket socket = null;
			this._serverSocket = new ServerSocket(this._serverSocketPort);
			while (this._stopServer == false) {
				socket = this._serverSocket.accept();
				
				long currentTime = System.currentTimeMillis();
				long differenceCooldown = currentTime - lastTimeOfCooldown;
				cooldown = cooldown - (int)differenceCooldown;
				if (  cooldown < 0 ) {
					cooldown = 0;
				}
				
				ClientSocketThread cst = new ClientSocketThread(getProductAgentID(), socket, agentHost, cooldown);
				cst.start();
				this.cooldown += cooldownLength;
				socket = null;
				cst = null;
				lastTimeOfCooldown = System.currentTimeMillis();
			}
		} catch (IOException ioe) {
				System.out.println("Exception: " + ioe.getMessage());
		}
	}
	
	private String getProductAgentID(){
		return "pa"+ productAgentID++;
	}
}