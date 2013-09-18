import java.io.BufferedReader;
import java.io.InputStreamReader;

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
	
	public final static boolean DEBUG = true;
	private static int paCnt = 10;
	private static String agentHost = "127.0.0.1";

	
	//Java -jar GatewayServer.jar 127.0.0.1
	public static void main(String[] args) {
		try {
			if(args != null && args.length == 1 && args[0].getClass() == String.class) {
				agentHost = (String)args[0];
			}
			GatewayServer gs = new GatewayServer(10081);
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
			
			System.out.println("Server Started!");
			gatewayServerThread.run();

		} catch (IllegalArgumentException uae) {
			System.out.println("Argument exception");
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("Common exception");
		}

	}
	
	public static String getAgentHost() {
		return agentHost;
	}
	
	public static int getPAID() {
		return paCnt++;
	}
}
