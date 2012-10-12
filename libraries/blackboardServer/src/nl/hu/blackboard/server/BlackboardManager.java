package nl.hu.blackboard.server;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import nl.hu.autokeystore.clientlib.AutoKeyStore;

import org.openbbs.blackboard.Blackboard;
import org.openbbs.blackboard.*;
import org.openbbs.blackboard.DefaultZone;
import org.openbbs.blackboard.ObjectBlackboard;
import org.openbbs.blackboard.ZonedBlackboardAccess;

// TODO: Auto-generated Javadoc
/**
 * The Class BlackboardManager.
 * Is the blackboardserver
 */
public class BlackboardManager 
{
	
	/** The blackboard. */
	private final Blackboard blackboard;
	
	/** The blackboard access. */
	private final ZonedBlackboardAccess	blackboardAccess;
	
	/** The default zone. */
	private DefaultZone defaultZone;
	
	/** The server socket. */
	private ServerSocket serverSocket;
	
	/** The connection. */
	private Socket connection;
	
	/** The auto key store client. */
	private AutoKeyStore autoKeyStoreClient;
		
	/**
	 * Instantiates a new blackboard manager.
	 */
	public BlackboardManager()  
	{		
		autoKeyStoreClient = new AutoKeyStore();
		blackboard = new ObjectBlackboard(new CloneBySerializationStrategy());
		defaultZone = new DefaultZone();
		blackboardAccess = new ZonedBlackboardAccess(blackboard, defaultZone);	
		try {
			serverSocket = new ServerSocket(Integer.parseInt(autoKeyStoreClient.getValue("blackboard.port")));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}			
	}
	
    /**
     * Handle connections.
     */
    public void handleConnections()
    {   
    	try {
			while(true)
			{
				System.out.println("Blackboard waiting for connection"); 			
				connection = serverSocket.accept();				
				connection.setSoLinger(false, 0);
				BlackboardThread newHandler = new BlackboardThread(blackboard, connection, blackboardAccess);
				Thread t = new Thread(newHandler);
				t.start();				
			}			
    	} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
    }  
}