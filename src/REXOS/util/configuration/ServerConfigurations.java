package util.configuration;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class ServerConfigurations {
	public static final String WSS_URI = "ws://127.0.0.1:8887";
	public static final String WSS_PORT = "8887";
	public static final String GS_IP = "";
	public static final String GS_PORT = "1099";
	public static final String AGENT_ADDRESS = getIP()+":"+GS_PORT+"/JADE";
	public static final String GS_NAME = "Grid@"+AGENT_ADDRESS;
	public static final String GS_ADDRESS = "http://" + getIP()+ ":1099";
	
	
	private static String getIP(){
		String ip ="";
		try {
		    Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
		    while (interfaces.hasMoreElements()) {
		        NetworkInterface iface = interfaces.nextElement();
		        // filters out 127.0.0.1 and inactive interfaces
		        if (iface.isLoopback() || !iface.isUp())
		            continue;
		
		        Enumeration<InetAddress> addresses = iface.getInetAddresses();
		        while(addresses.hasMoreElements()) {
		            InetAddress addr = addresses.nextElement();
		            ip = addr.getHostAddress();
		            System.out.println(iface.getDisplayName() + " " + ip);
		        }
		    }
		} catch (SocketException e) {
			System.out.println("Exception!");
		    throw new RuntimeException(e);
		    
		}
		return ip;
	}
}


