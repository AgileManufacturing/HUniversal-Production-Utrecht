package gatewayserver;

import gatewayserver.data.Command;
import jade.cli.CLIManager;
import jade.util.ExtendedProperties;
import jade.util.leap.Properties;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

/**
 *
 * Project: GatewayServer
 *
 * Package: 
 *
 * File: ClientSocketThread.java
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
public class ClientSocketThread implements Runnable {

	private GatewayServer server; 
	
	private boolean _stopSocketThread = false;
	private Socket _clientSocket = null;
	private BufferedReader _clientInSocket = null;
	
	private String agentHost;

	public ClientSocketThread(GatewayServer server, Socket socket, String agentHost) {
		this.server = server;
		this.agentHost = agentHost;
		this._clientSocket = socket;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		try {
			//this._clientOutSocket = new PrintWriter(
			//		this._clientSocket.getOutputStream(), true);
			this._clientInSocket = new BufferedReader(new InputStreamReader(
					this._clientSocket.getInputStream()));
			Command cmd = null;
			JsonParser parser = new JsonParser();
			while(this._stopSocketThread == false) {
				try{
					String data = this._clientInSocket.readLine();
					JsonObject obj = (JsonObject)parser.parse(data).getAsJsonObject();
			        JsonElement idElement = obj.get("id");
			        JsonElement commandElement = obj.get("command");
			        JsonElement  productElement = obj.get("payload");
			        
			        int id = idElement.getAsInt();
			        String command = commandElement.getAsString();
			        String payload = productElement.toString();
			        
			        cmd = new Command(id,command,payload);
					
					if(cmd.getCommand().equals("CREATE_PA")) {
						String json = cmd.getPayload();											
						CommandAgent ca = new CommandAgent(json, server.getProductAgentID());
						Properties pp = new ExtendedProperties();
						pp.put("host", agentHost);
						
						CLIManager.execute(pp, ca.getBehaviour(pp), false);
					}
					System.out.println("Sent new product agent to JADE.");
					stop();
				}
				catch(Exception e) {
					e.printStackTrace();
					stop();
				}				
			}
		} catch (Exception e) {
			System.out.println("Exception: " + e.getMessage());
			stop();
		}
	}

	public void stop() {
		this._stopSocketThread = true;
		try {
			_clientSocket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}