package SCADA;

import java.net.URI;
import java.net.URISyntaxException;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

public class WebSocketClientTest extends WebSocketClient {

	public WebSocketClientTest(URI serverURI) {
		super(serverURI);
	}

	@Override
	public void onClose(int arg0, String arg1, boolean arg2) {
		System.out.println("Closed connection!");
	}

	@Override
	public void onError(Exception arg0) {
		arg0.printStackTrace();
	}

	@Override
	public void onMessage(String arg0) {
		System.out.println(arg0);
	}

	@Override
	public void onOpen(ServerHandshake arg0) {
		System.out.println("Opened connection!");
	}
	
	
	public static void main(String[] args) {
		try {
			WebSocketClientTest mws = new WebSocketClientTest(new URI("ws://127.0.0.1:3529"));
			mws.connect();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			mws.send("THIS IS A TEST MESSAGE!");
		} catch (URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
