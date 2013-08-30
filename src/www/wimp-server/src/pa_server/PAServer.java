package pa_server;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;

public class PAServer extends ServerSocket  implements Runnable{
		
		public PAServer() throws Exception {
			
		}

		public void removeService() {
			//this.services.remove(service);
			//this.threads.remove(service);
		}

		@Override
		public void run() {
			System.out.println("webservert starting...");
			String clientSentence;
			ServerSocket welcomeSocket = null;

			try {
				welcomeSocket = new ServerSocket(9999);
			}
			catch (Exception e){
				e.printStackTrace();
			}

			while (true) {
				
				try{
				Socket connectionSocket = welcomeSocket.accept();
				BufferedReader inFromClient = new BufferedReader(
						new InputStreamReader(
								connectionSocket.getInputStream()));
				DataOutputStream outToClient = new DataOutputStream(
						connectionSocket.getOutputStream());

				clientSentence = inFromClient.readLine();

				System.out.println("Received: " + clientSentence);

				outToClient.writeBytes("{\"error\": false, \"msg\":\"RECEIVED\"}");

			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			}
		}

		public void stop() {//TODO
/*
			for (Service s : services) {
				s.closeSocket();
				removeService(s);
			}

			try {
				this.close();
			} catch (IOException e) {
				e.printStackTrace();
			}*/
		}
	}

