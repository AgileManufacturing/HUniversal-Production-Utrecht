package MAS.testerClasses;

import org.json.JSONTokener;
import org.json.JSONWriter;
import org.json.JSONObject;
import org.json.JSONException;

import java.io.FileNotFoundException;
import java.io.FileInputStream;
import java.io.File;
import java.util.Iterator;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import org.java_websocket.WebSocket;
import org.java_websocket.WebSocketImpl;
import org.java_websocket.framing.Framedata;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;


public class TestSocketServer extends WebSocketServer {
	/**
	 * The JSONObject of this class
	 */
	private JSONObject object;

	/**
	 * Constructor, initializing the listening socket
	 * @param  port          The port number to listen on
	 * @throws UnknownHostException 
	 * @throws JSONException [description]
	 * @throws FileNotFoundException   [description]
	 */
	public TestSocketServer(int port) throws UnknownHostException {
		super(new InetSocketAddress(Inet4Address.getLocalHost(),port));
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
		
		if (message.equals("get json")) {
			conn.send(this.object.toString());
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
