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

	private ArrayList<ClientSocketThread> _clientThreads;

	private boolean _stopServer = false;

	private int productAgentID = 10;
	
	private String agentHost = "";
	
	public GatewayServer(int port, String agentHost) throws IOException {
		super();
		this.agentHost = agentHost;
		this._serverSocketPort = port;
		_clientThreads = new ArrayList<ClientSocketThread>();
	}

	public void run() {
		try {
			Socket socket = null;
			Thread runThread = null;
			this._serverSocket = new ServerSocket(this._serverSocketPort, 10);
			while (this._stopServer == false) {
				socket = this._serverSocket.accept();
				ClientSocketThread cst = new ClientSocketThread(this, socket, agentHost);
				runThread = new Thread(cst);
				runThread.start();
			}
		} catch (IOException ioe) {
				System.out.println("Exception: " + ioe.getMessage());
		} catch (Exception e) {
				System.out.println("Exception: " + e.getMessage());
		}
	}

	public void stopServer() {
		this._stopServer = true;
		for (ClientSocketThread sct : this._clientThreads) {
			sct.stop();
		}
	}

	
	public String getProductAgentID(){
		return "pa"+ productAgentID++;
	}
}