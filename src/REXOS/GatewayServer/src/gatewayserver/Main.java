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