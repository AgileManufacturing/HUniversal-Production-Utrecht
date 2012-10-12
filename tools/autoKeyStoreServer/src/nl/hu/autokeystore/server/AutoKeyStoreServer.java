//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        AutoKeyStoreServer
// File:           AutoKeyStoreServer.java	  
// Description:    AutoKeyStore is a server that will multicast it's existence 
//                  over the LAN and offers a service for retrieving values. 
// Author:         Pascal Muller
// Notes:          
//
// License:        newBSD
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

/**
 * The AutoKeyStoreServer class is responsible for parsing command line options and 
 * starting an instance of AutoConfServer and KeyValueServer. 
 * @author pascal
 */
public final class AutoKeyStoreServer
{
    private static Storage store;

    /**
     * @param args The command line arguments
     */
    public static void main(String[] args)
    {
        try
        {
            int port = 4447; //Default port for the KeyValueServer
            String storageFile = "keyvalDB"; //Default file with data to be served
            boolean writeTestMode = false; 

            int i = 0;
            while (i < args.length)
            {
                if (args[i].equals("-p"))
                {
                    port = Integer.parseInt(args[++i]); //set port number
                }
                else if (args[i].equals("-f"))
                {
                    storageFile = args[++i]; //set storagefile
                }
                else if (args[i].equals("-w"))
                {
                    writeTestMode = true;
                }
                else
                {
                    throw new IllegalArgumentException("Unrecognized arguments");
                }
                i++;
            }

            store = new Storage(storageFile);
            if (writeTestMode)
            {
                //In write test mode, the internal storage is written to a file.  
                store.dump(storageFile);
                System.exit(0);
            }

            //Creating an instance of AutoKeyStoreServer that will start both 
            //of our services. 
            AutoKeyStoreServer akss = new AutoKeyStoreServer(4445, port); 
            //Because the client library does not support changing the autodiscovery 
            //port, it is set to 4445 here by default. 
        }
        catch (Exception e)
        {
            System.err.println(e.getMessage());
            //TODO, print usage info??
            System.exit(-1);
        }
    }

    /**
     * Constructor for AutoKeyStoreServer. Is responsible for starting instances 
     * of the two services, both running in separate threads. 
     * @param discoverPort Port where the automatic configuration service 
     * (AutoConfServer) is listening
     * @param servicePort Port where the key-value service (KeyValueServer) is 
     * listening
     */
    public AutoKeyStoreServer(int discoverPort, int servicePort)
    {
        this.startServices(discoverPort, servicePort);
    }

    /**
     * This function is responsible for starting instances of the two services, 
     * both running in separate threads.
     * @param discoverPort Port where the automatic configuration service 
     * (AutoConfServer) is listening
     * @param servicePort Port where the key-value service (KeyValueServer) is 
     * listening
     */
    public void startServices(int discoverPort, int servicePort)
    {
        try
        {
            AutoConfServer autoConfServer = new AutoConfServer(discoverPort, servicePort);
            Thread thread = new Thread(autoConfServer);
            KeyValueServer keyValServer = new KeyValueServer(servicePort, store);
            Thread thread2 = new Thread(keyValServer);
            
            //Wait forever.. (...actually, until the server is killed or the 
            //threads throw an exception for some reason)
            thread.start();
            thread2.start();
            thread.join();
            thread2.join();
        }
        catch (Exception ex)
        {
            System.err.println(ex.getMessage());
            System.exit(-1);
        }
    }
}
