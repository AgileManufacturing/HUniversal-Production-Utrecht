package web_socket_server.java.org.java_websocket.handshake;

public interface HandshakeBuilder extends Handshakedata {
	public abstract void setContent( byte[] content );
	public abstract void put( String name, String value );
}
