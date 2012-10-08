/**
* @file AutoKeyStore.java
* @brief Can be used by Java programs to access values of keys on the AutoKeyStore configuration server.
*
* @author Pascal Muller
*
* @section LICENSE
* Copyright © 2012, HU University of Applied Sciences Utrecht.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

package nl.hu.autokeystore.clientlib;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.*;

/**
 * The AutoKeyStore class is a library class that can be used by Java programs
 * to access values of keys on the AutoKeyStore configuration server. 
 * 
 * The server multicasts a packet indicating its presence and offers and 
 * a service to retrieve configuration values. This library hides the 
 * implementation details from the user. The library takes care of discovering 
 * the server and asking for configuration values. The developer asks this 
 * library for one or more configuration values without having to comprehend the 
 * black voodoo magic(c) that powers this functionality. 
 * 
 * @author Pascal
 */
public class AutoKeyStore
{

    /**
     * The servers IP address in ASCII
     */
    String ip = null;
    /**
     * The servers port number (for the key-value retrieval service)
     */
    int port = -1;
    
    /**
     * The cache object allows for retrieving cached copies of the 
     * data (of the key-value retrieval service)
     */
    Cache cache;

    /**
     * Constructor for the AutoKeyStore object, which offers the API. 
     * 
     * We should think about offering the option to use this API in a
     * non-blocking fashion. However, because of the implementation of our
     * agents, the current implementation only offers blocking functions.
     */
    public AutoKeyStore()
    {
        //At first, I thought about doing the discovery of the server here, 
        //however, I decided to keep the constructor short and fast. On the 
        //first request this will be done before doing the actual request. 
        //If performance is an issue, it should be simple to implement a persistent
        //connection, however, because configuration data requests are small and only 
        //done on startup, this shouldn't be an issue.    
        
        //Create new cache for key-value stores.
        cache = new Cache();
    }

    /**
     * Gets a single value from the server. For performance reasons, it is 
     * recommended to use getValues() instead of multiple calls to getValue() if 
     * this is possible. 
     * 
     * @param key The key. 
     * @return The (String) value that corresponds with the key.
     * @throws RuntimeException When the connection to the server fails for 
     * whatever reason, an exception is thrown. 
     */
    public String getValue(String key) throws RuntimeException
    {
        String[] inVal =
        {
            key
        };
        //getValue() is just a wrapper around getValues()
        String[] retVal = getValues(inVal);
        return retVal[0];
    }

    /**
     * Retrieves multiple values from the server at once. This function keeps the 
     * connection open when requesting multiple values. It is therefore recommended 
     * to use a single call to this method instead of successive calls to getValue().
     * 
     * @param keys The keys. 
     * @return The values that correspond with the keys. 
     * @throws RuntimeException When the connection to the server fails for 
     * whatever reason, an exception is thrown. 
     */
    public String[] getValues(String[] keys) throws RuntimeException
    {
        //Check if the server is already discovered or discover the server. 
        checkServer();
        
        String retVal[] = new String[keys.length];
        boolean allCached = cache.checkCache(keys, retVal);
        try
        {
            //If everything is still cached, creating a new connection and 
            //retrieving all values slows down the program. 
            if(!allCached) 
            {
                //Create connection
                Socket s = new Socket(ip, port);
                BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream()));
                PrintWriter out = new PrintWriter(s.getOutputStream());

                //For all keys
                for (int i = 0; i < keys.length; i++)
                {
                    out.print(keys[i] + "\n");
                    out.flush();
                    String answer = null;
                    //get value of key
                    if ((answer = in.readLine()) != null)
                    {
                        //if the answer equals "NULL", it should be set to null. 
                        retVal[i] = (answer.equals("NULL")) ? null : answer;
                        //update the cache
                        cache.updateCacheEntry(keys[i], retVal[i]);
                    }
                }
                //CLose connection
                in.close();
                out.close();
                s.close();
            }
        }
        catch (Exception ex)
        {
            //If any exception is thrown during this process, inform the user
            throw new RuntimeException("AutoKeyStore: Error connecting to server");
        }
        return retVal;
    }

    /**
     * Checks if the server is already discovered and attempts to discover the server. 
     * 
     * If the server was already discovered, this method immediately stops. 
     * If it isn't, an attempt is made to discover the server. If this fails, the 
     * environment variable AKSS_MASTER_URI is read and used. Should this fail, a 
     * RuntimeException is thrown to inform the user. 
     */
    private void checkServer()
    {
        //If the server is not yet discoverd
        if (port == -1 || ip == null)
        {
            //If the server cannot be discoverd in the normal way
            if (!discoverServer())
            {
                String master_uri = System.getenv("AKSS_MASTER_URI");
                //Try using the AKSS_MASTER_URI environment variable
                if (master_uri != null && parseRL(master_uri))
                {
                    //succes
                }
                else
                {
                    System.out.println("WARNING: AKSS_MASTER_URI not set or malformed: " + master_uri);
                    throw new RuntimeException("AutoKeyStore: Error discovering server");
                }
            }
        }
    }

    /**
     * discoverServer() listens to multicast packets sent by the AutoKeyStore server.
     * @return true if server is successfully discovered and the packet sent by the 
     * server adheres to the correct format. 
     */
    private boolean discoverServer()
    {
        boolean serverFound = false;
        try
        {
            MulticastSocket socket = new MulticastSocket(new InetSocketAddress(4446));
            InetAddress address = InetAddress.getByName("239.255.255.250");
            socket.joinGroup(address);
            //Server sends at 500 ms intervals, waiting 750ms was emperically 
            //chosen because most of the time the next package seems to be 
            //ready to be received. 
            socket.setSoTimeout(750); 
            DatagramPacket packet;
            
            //Try receiving the packet twice
            int tries = 2;
            while (tries > 0)
            {
                try
                {
                    byte[] buf = new byte[256];
                    packet = new DatagramPacket(buf, buf.length);
                    socket.receive(packet);

                    String received = new String(packet.getData(), 0, packet.getLength());

                    if (received.substring(0, 4).equals("AKSS"))
                    {
                        //The packet starts with our identifier and contains an ip and port
                        serverFound = parseRL(received.substring(5)); //Succesfull
                        break; //And done :-)
                    }
                }
                catch (SocketTimeoutException e)
                {
                    //Only decrease tries if timeout was exceeded, because on a busy 
                    //network, lots of packages might be received that are not sent by 
                    //our server. 
                    tries--;
                }
            }

            socket.leaveGroup(address);
            socket.close();
        }
        catch (IOException e)
        {
            //serverFound will be false and this function will end
        }
        return serverFound;
    }
    
    /**
     * Parse the URL and port number for a resource locator that indicates the 
     * IP and port of the key-value storage service. 
     * @param rl S resource locator that indicates the IP and port of the 
     * key-value storage service.
     * @return True if rl could be parsed and this.port and this.ip were 
     * succesfully set, false otherwise. 
     */
    private boolean parseRL(String rl)
    {
        String[] split = rl.split(":");
        if (split.length == 2)
        {
            ip = split[0]; //retrieve ip
            try
            {
                port = Integer.parseInt(split[1], 10);  //and port
            }
            catch (NumberFormatException e){}            
        }
        if(port == -1 || ip== null)
        {
            //Set both to their default values
            ip = null;
            port = -1;
            //and return false;
            return false;
        }
        return true;
    }
}
