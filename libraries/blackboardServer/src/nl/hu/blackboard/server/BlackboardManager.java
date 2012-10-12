//******************************************************************************
//
//                 Blackboard Server
//
//******************************************************************************
// Project:        Blackboard Server
// File:           BlackboardManager.java	  
// Description:    ?
// Author:         Pascal Muller
// Notes:          
//
// License:        newBSD
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

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