package pa_server;

import java.io.IOException;
import java.io.OutputStream;
import java.net.SocketException;
import java.util.HashMap;

class Request extends HashMap<String, String> {
	private static final long serialVersionUID = 1L;
	private final SocketInputStream sis;
	private PACommunicationServer server;
	
	public Request(SocketInputStream sis, OutputStream os, PACommunicationServer server) {
		this.sis = sis;
		this.server = server;

		String requestLine = readLine();
		
		if(requestLine == null){
			this.server.mmib.sendMessage("{'error':true, 'message':'Received null from PA. Connection closed.'}");
			try {
				sis.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return;
		}
		
		System.out.println("readline: " + requestLine);
		this.server.mmib.sendMessage("{'error':false, 'message':'Received from PA: "+requestLine+"'}");
		
		String s = "{'error':false, 'message':'message received: "+ requestLine +"'}\r\n";
		try {
			os.write(s.getBytes());
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
	}

	public String[] readLines() throws Exception {
		String[] lines = new String[50];
		int i = 0;
		String line = "";

		while ((line = sis.readLine()) != null) {
			lines[i] = line;
			i++;
		}

		return lines;
	}

	public String readLine() {
		try {
			return sis.readLine();
		} catch (SocketException e) {
		} catch (Exception e) {
			//e.printStackTrace();
		} finally {

		}
		return null;
	}

	

}
