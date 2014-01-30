package gatewayserver;

import gatewayserver.data.Command;
import jade.cli.CLIManager;
import jade.util.ExtendedProperties;
import jade.util.leap.Properties;
import jade.wrapper.ControllerException;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

import javax.management.monitor.Monitor;

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
public class ClientSocketThread extends Thread implements Runnable {

	private String productAgentId;

	private boolean _stopSocketThread = false;
	private Socket _clientSocket = null;
	private BufferedReader _clientInSocket = null;

	private String agentHost;
	private int cooldown;

	public ClientSocketThread(String productAgentId, Socket socket,
			String agentHost, int cooldown) {
		this.productAgentId = productAgentId;
		this.agentHost = agentHost;
		this._clientSocket = socket;
		this.cooldown = cooldown;
	}

	@Override
	public void run() {
		try {
			this._clientInSocket = new BufferedReader(new InputStreamReader(
					this._clientSocket.getInputStream()));

			Command cmd = null;
			JsonParser parser = new JsonParser();
			String data = this._clientInSocket.readLine();
			JsonObject obj = (JsonObject) parser.parse(data).getAsJsonObject();
			JsonElement idElement = obj.get("id");
			JsonElement commandElement = obj.get("command");
			JsonElement productElement = obj.get("payload");

			int id = idElement.getAsInt();
			String command = commandElement.getAsString();
			String payload = productElement.toString();
			
			cmd = new Command(id, command, payload);
			
			stopClient();
			
			if (cmd.getCommand().equals("CREATE_PA")) {
				String json = cmd.getPayload();
				if (cooldown > 0) {
					System.out.println("Starting new productAgent after: " + cooldown + "milliseconds");
					try {
						Thread.sleep(cooldown);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				CommandAgent ca = new CommandAgent(json, productAgentId);
				Properties pp = new ExtendedProperties();
				pp.put("host", agentHost);

				try {
					CLIManager.execute(pp, ca.getBehaviour(pp), false);
				} catch (IllegalArgumentException | ControllerException e) {
				}
			}
			System.out.println("Sent new product agent to JADE.");
			

		} catch (IOException e1) {
			e1.printStackTrace();
		}
	}

	public void stopClient() {
		this._stopSocketThread = true;
		try {
			_clientSocket.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}