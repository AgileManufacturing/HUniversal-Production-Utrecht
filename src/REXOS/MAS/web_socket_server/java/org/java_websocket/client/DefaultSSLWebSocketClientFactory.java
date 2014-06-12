package web_socket_server.java.org.java_websocket.client;
import java.io.IOException;
import java.net.Socket;
import java.nio.channels.ByteChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.SocketChannel;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLEngine;

import web_socket_server.java.org.java_websocket.SSLSocketChannel2;
import web_socket_server.java.org.java_websocket.WebSocketAdapter;
import web_socket_server.java.org.java_websocket.WebSocketImpl;
import web_socket_server.java.org.java_websocket.client.WebSocketClient.WebSocketClientFactory;
import web_socket_server.java.org.java_websocket.drafts.Draft;



public class DefaultSSLWebSocketClientFactory implements WebSocketClientFactory {
	protected SSLContext sslcontext;
	protected ExecutorService exec;

	public DefaultSSLWebSocketClientFactory( SSLContext sslContext ) {
		this( sslContext, Executors.newSingleThreadScheduledExecutor() );
	}

	public DefaultSSLWebSocketClientFactory( SSLContext sslContext , ExecutorService exec ) {
		if( sslContext == null || exec == null )
			throw new IllegalArgumentException();
		this.sslcontext = sslContext;
		this.exec = exec;
	}

	@Override
	public ByteChannel wrapChannel( SocketChannel channel, SelectionKey key, String host, int port ) throws IOException {
		SSLEngine e = sslcontext.createSSLEngine( host, port );
		e.setUseClientMode( true );
		return new SSLSocketChannel2( channel, e, exec, key );
	}

	@SuppressWarnings("deprecation")
	@Override
	public WebSocketImpl createWebSocket( WebSocketAdapter a, Draft d, Socket c ) {
		return new WebSocketImpl( a, d, c );
	}

	@SuppressWarnings("deprecation")
	@Override
	public WebSocketImpl createWebSocket( WebSocketAdapter a, List<Draft> d, Socket s ) {
		return new WebSocketImpl( a, d, s );
	}
}