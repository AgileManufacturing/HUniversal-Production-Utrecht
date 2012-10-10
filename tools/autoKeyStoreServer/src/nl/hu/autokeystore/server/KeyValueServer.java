//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        AutoKeyStoreServer
// File:           KeyValueServer.java	  
// Description:    AutoKeyStore is a server that will multicast it's existence 
//                  over the LAN and offers a service for retrieving values. 
// Author:         Pascal Muller
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of AutoKeyStore.
//
// AutoKeyStoreServer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// AutoKeyStoreServer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with AutoKeyStoreServer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
package nl.hu.autokeystore.server;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * The KeyValueServer class implements the key-value storage. It listens to 
 * requests for one or more values that correspond to a server
 * port the KeyValueServer is listening on. 
 * @author pascal
 */
public class KeyValueServer implements Runnable
{

    private ServerSocket socket;
    private Storage storage;

    /**
     * Constructor, creates a ServerSocket for listening to requests.
     * @param servicePort The port on which the service should listen. 
     * @param storage An instance of Storage that is used to store key-value pairs. 
     * @throws IOException Throws an IOException if creating a ServerSocket for 
     * listening fails.  
     */
    public KeyValueServer(int servicePort, Storage storage) throws IOException
    {
        socket = new ServerSocket(servicePort);
        this.storage = storage;
    }

    /**
     * The run function is ran as a separate thread and is responsible for 
     * listening to requests.
     */
    @Override
    public void run()
    {
        while (true) 
        {
            try
            {
                Socket clientSocket = socket.accept(); //Accept clients
                new Thread(new ClientHandler(clientSocket)).start(); //and start a thread to handle those
            }
            catch (Exception ex)
            {
                //Let's just retry
                System.out.println("Warning: " + ex.getMessage());
            }
        }
    }

    /**
     * The ClientHandler class implements the functionality needed to handle and 
     * respond to a request. 
     */
    private class ClientHandler implements Runnable
    {

        private Socket clientSocket;

        /**
         * Constructor.
         * @param clientSocket A socket that is connected to a client. 
         */
        public ClientHandler(Socket clientSocket)
        {
            this.clientSocket = clientSocket;
        }

        /**
         * This method is responsible for handling and responding to requests over 
         * the clientSocket. 
         * 
         * The protocol is pretty simple: (Note: the quotes are not a part of the data sent)
         * LOOP
         *  client -> server: "KEY\n"
         *  server -> client: "VALUESTRING\n" //VALUESTRING is either the requested 
         *                                      value or a string containing NULL
         * ENDLOOP if the connection is closed by the client. 
         */
        @Override
        public void run()
        {
            try
            {
                BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                PrintWriter out = new PrintWriter(clientSocket.getOutputStream());
                String command;
                while ((command = in.readLine()) != null) //Read till \n
                {
                    if (storage.exists(command))
                    {
                        out.println(storage.get(command)); //write with \n at the end (print*LN*!)
                    }
                    else
                    {
                        out.println("NULL"); //write with \n at the end (print*LN*!)
                    }
                    //Flush answer, because client will wait with the next key till the data is in. 
                    out.flush(); 
                }
                in.close();
                out.close();
                clientSocket.close();
            }
            catch (IOException e)
            {
                //report exception somewhere.
                e.printStackTrace();
            }
        }
    }
}
