package web_socket_server.java.org.java_websocket;

import java.net.Socket;
import java.util.List;

import web_socket_server.java.org.java_websocket.drafts.Draft;


public interface WebSocketFactory {
	public WebSocket createWebSocket( WebSocketAdapter a, Draft d, Socket s );
	public WebSocket createWebSocket( WebSocketAdapter a, List<Draft> drafts, Socket s );

}
