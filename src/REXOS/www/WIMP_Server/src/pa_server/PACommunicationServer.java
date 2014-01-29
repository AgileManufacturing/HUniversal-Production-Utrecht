package pa_server;

import java.io.IOException;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.CharBuffer;
import java.util.HashSet;
import servlets.ProductServlet.MyMessageInbound;

public class PACommunicationServer extends ServerSocket implements Runnable {
	private final HashSet<Thread> threads = new HashSet<Thread>();
	private final HashSet<Service> services = new HashSet<Service>();
	
	private boolean running = true;
	
	MyMessageInbound mmib;

	public PACommunicationServer(InetAddress bindAddress, int port, MyMessageInbound mmib)
			throws Exception {
		super(8888, 50, bindAddress);
		this.mmib = mmib;
	}

	public boolean isAlive() {
		return !services.isEmpty();
	}
	
	public boolean isRunning(){
		return running;
	}

	public void removeService(Service service) {
		this.services.remove(service);
		this.threads.remove(service);
	}

    public void terminate() {
    	
        running = false;
    }
    
	@Override
	public void run() {
		System.out.println("server started");
		
		mmib.sendMessage("{'error': false, 'message':'Server started on port "+ this.getLocalPort() +"'}");
		while (running) {
			try {
				Socket socket = this.accept();
				System.out.println("new connection");
				mmib.sendMessage("{'error': false, 'message':'PA server: New connection'}");
				Service service = new Service(socket, this.services.size() + 1, this);
				services.add(service);

				Thread thread = new Thread(service);
				threads.add(thread);

				thread.start();
			} catch (Exception e) {
				// control.log(e.getMessage(), 0);
				mmib.sendMessage("{'error': true, 'message':'Server problem!'}");
				System.out.println(e.getMessage());
			}
		}
		//stop();
	}

	public void stop() {
		System.out.println("stop()");
		for (Service s : services) {
			s.closeSocket();
			removeService(s);
		}

		try {
			this.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		System.out.println("Connections stopped");
	}
}
