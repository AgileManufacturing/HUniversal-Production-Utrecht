//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        AutoKeyStoreServer
// File:           AutoConfServer.java	  
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

import java.io.IOException;
import java.net.*;
import java.util.Collections;
import java.util.Enumeration;

/**
 * The AutoConfServer class implements the automatic configuration service. It 
 * multicasts its existence by sending out the IP-address of this system and the 
 * port the KeyValueServer is listening on. 
 * @author pascal
 */
public class AutoConfServer implements Runnable
{

    private int discoverPort;
    private int servicePort;
    private MulticastSocket socket;
    private String ip;

    /**
     * Constructor. The constructor starts the service
     * @param discoverPort Port where the automatic configuration service 
     * (AutoConfServer) is listening
     * @param servicePort Port where the key-value service (KeyValueServer) is 
     * listening
     * @throws IOException If the multicastsocket could not be instantiated 
     * or an IPv4-address could not be found. 
     */
    public AutoConfServer(int discoverPort, int servicePort) throws IOException
    {
        this.discoverPort = discoverPort;
        this.servicePort = servicePort;
        socket = new MulticastSocket(new InetSocketAddress(this.discoverPort));
        socket.setTimeToLive(32);
        ip = getServerIP();
        if (ip == null)
        {
            throw new IOException("The system has no IPv4 adresses configured. ");
        }
    }

    /**
     * The run function is ran as a separate thread and is responsible for sending 
     * out the multicast packets at regular intervals. 
     */
    @Override
    public void run()
    {
        while (true)
        {
            try
            {
                byte[] buf;
                String dString = "AKSS-" + ip + ":" + servicePort;
                buf = dString.getBytes();

                // send it
                InetAddress group = InetAddress.getByName("239.255.255.250");
                DatagramPacket packet = new DatagramPacket(buf, buf.length, group, 4446);
                socket.send(packet);
                Thread.sleep(500);
            }
            catch (Exception ex)
            {
                ex.printStackTrace();
            }
        }
    }

    /**
     * Returns the first IPv4 address belonging to this machine. 
     * @return The first IPv4 address or null if this couldn't be determined. 
     */
    public static String getServerIP()
    {
        try
        {
            //For all network interfaces.. 
            for (NetworkInterface networkInterface : Collections.list(NetworkInterface.getNetworkInterfaces()))
            {
                //..that aren't loopback, down, or a VirtualBox interface
                if (((!networkInterface.isLoopback()) && networkInterface.isUp())
                        && !networkInterface.getDisplayName().equals("VirtualBox Host-Only Ethernet Adapter"))
                    //The Virtualbox-adapter didn't work correctly for multicast packets. 
                {
                    //For all IP addresses  assigned to this interface
                    for (Enumeration<InetAddress> e = networkInterface.getInetAddresses(); e.hasMoreElements();)
                    {
                        InetAddress nextElement = e.nextElement();
                        if (nextElement instanceof Inet4Address)
                        {
                            //return first address on first networkcard that is applicable
                            return nextElement.getHostAddress(); 
                        }
                    }
                }
            }
            return null;
        }
        catch (SocketException e)
        {
            e.printStackTrace();
        }
        return null;
    }
}
