package gatewayserver;

import java.io.IOException;

/**
 *
 * Project: GatewayServer
 *
 * Package: 
 *
 * File: Main.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */

/**
 * @author Mike
 * 
 */
public class Main {
	
	private static String agentHost = "127.0.0.1";
	private static int port = 10081;
	
	//usage: Java -jar GatewayServer.jar 127.0.0.1
	
	public static void main(String[] args) {
		try {
			if(args != null && args.length == 1 && args[0].getClass() == String.class) {
				agentHost = (String)args[0];
			}
			GatewayServer gs = new GatewayServer(port, agentHost);
			Thread gatewayServerThread = new Thread(gs);
			
			/*
			InputStreamReader isr = new InputStreamReader(System.in);
			BufferedReader br = new BufferedReader(isr);
			String line = "";
			System.out.println("Typ 'Start' to run the GatewayServer");
			while ((line = br.readLine()) != null) {
				String option = line.trim().toLowerCase();
				if(option.equals("start")){
					System.out.println("Server is starting!");
					gatewayServerThread.run();
				} else if(option.equals("stop")) {
					System.out.println("Server is stopping!");
					gs.stopServer();
				} else if(option.equals("quit")) {
					gs.stopServer();
					System.exit(0);		
				} else {
					System.out.println("Unknown command");
				}
			}
			*/
			
			System.out.println("GatewayServer Started on: " + agentHost + ":" + port);
			gatewayServerThread.run();

		} catch (IllegalArgumentException uae) {
			System.out.println("Argument exception");
			uae.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.println("Could not start the gatewayserver");
			e.printStackTrace();
		} 

	}
}