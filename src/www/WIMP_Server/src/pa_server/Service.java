package pa_server;

import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;

class Service implements Runnable {
	private Socket socket = null;

	private SocketInputStream sis;
	private OutputStream os;

	private final int id;
	private Servlet servlet;
	
	private boolean running = true;

	private final PACommunicationServer server;

	public Service(Socket socket, int id, PACommunicationServer server) {

		this.socket = socket;
		try {
			sis = new SocketInputStream(socket.getInputStream());
			os = socket.getOutputStream();
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}

		this.id = id;

		this.servlet = new Servlet(id);
		this.server = server;
	}

	public void terminate() {
        running = false;
    }
	
	public void closeSocket() {
		try {
			socket.close();
		} catch (IOException e) {

		}
	}

	@Override
	public void run() {
		try {
			while (running) {
				if (!socket.isClosed()) {
					Request request = new Request(sis, os, this.server);
					
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
			// control.log(e.getMessage(), id);
		} finally {
			this.closeSocket();
			//server.removeService(this);
		}
	}
}
