package MAS.testerClasses;

import org.json.JSONTokener;
import org.json.JSONObject;
import org.json.JSONException;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileInputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

public class TestSocketServer extends WebSocketServer {
	/**
	 * The JSONObject of this class
	 */
	private JSONObject object;
	
	private ArrayList<Message> messages;

	/**l
	 * Constructor, initializing the listening socket
	 * @param  port          The port number to listen on
	 * @throws UnknownHostException 
	 * @throws JSONException [description]
	 * @throws FileNotFoundException   [description]
	 */
	public TestSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(port));
		messages = new ArrayList<Message>();
	}

	public void loadJson(String filepath) throws JSONException, FileNotFoundException {
		File            file              = null;
		FileInputStream file_input_stream = null;
		JSONTokener     json_tokener      = null;
		JSONObject      object            = null;

		file              = new File(filepath);
		file_input_stream = new FileInputStream(file);
		json_tokener      = new JSONTokener(file_input_stream);
		object            = new JSONObject(json_tokener);

		this.object = object;
	}


	@Override
	public void onOpen(WebSocket conn, ClientHandshake handshake) {
		System.out.println(conn.getRemoteSocketAddress().getAddress().getHostAddress() + " connected");
	}

	@Override
	public void onClose(WebSocket conn, int code, String reason, boolean remote) {
		System.out.println(conn.getRemoteSocketAddress().getAddress().getHostAddress() + " left");
	}

	@Override
	public void onMessage(WebSocket conn, String message) {
		System.out.println(conn.getRemoteSocketAddress().getAddress().getHostAddress() + ": " + message);
		JSONObject command;
		try {
			command = new JSONObject(message);
			String agentName = command.getString("name");
			int messageID = command.getInt("id");
			long time = command.getLong("time");
			long timeNow = System.currentTimeMillis();
			
			System.out.println("Name: " + agentName + " ID: " + messageID + " time: " + time + " time now: " + timeNow);
			Message newMessage = new Message(agentName, messageID, time, timeNow);
			messages.add(newMessage);
			
			//500 agents and 5 messages. Yes this is ugly
			if(messages.size() == (500 * 500)){
				logToFile("WebSocketTest.csv");
			}
		} catch (JSONException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
/*		if (message.equals("get json")) {
			conn.send(this.object.toString());
		}*/
	}
	
	public void logToFile(String filename){
		try {
			FileWriter fw = new FileWriter(filename);
			BufferedWriter bf = new BufferedWriter(fw);
			bf.write("Name; ID; time send; time received; totalTime\n");
			for(int i = 0; i < messages.size(); i++){
				bf.write(messages.get(i).getName() + "; " + messages.get(i).getID() + "; " + messages.get(i).getTimeSend() + "; " + messages.get(i).getTimeReceived() + "; " + (messages.get(i).getTimeReceived()-messages.get(i).getTimeSend())+"\n");
			}
			bf.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	// Javascript API does not use fragments, so this won't be used.
	/*@Override
	public void onFragment(WebSocket conn, Framedata fragment) {
		System.out.println("received fragment: " + fragment);
	}*/			

	@Override
	public void onError(WebSocket conn, Exception ex) {
		ex.printStackTrace();
		if(conn != null) {

		}
	}

}
