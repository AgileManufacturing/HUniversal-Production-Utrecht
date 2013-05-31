/**
 * 
 * Project: Dummy Product Agent
 * 
 * Package: productAgent
 * 
 * File: SocketBehaviour.java
 * 
 * Author: Mike Schaap
 * 
 * Version: 1.0
 * 
 */

package rexos.mas.productAgent;

import jade.core.behaviours.CyclicBehaviour;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

import rexos.libraries.log.Logger;

public class SocketBehaviour extends CyclicBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Socket socket;
	private PrintWriter out = null;
	private BufferedReader in = null;

	public SocketBehaviour(){
		try{
			socket = new Socket(InetAddress.getByName("127.0.0.1"), 10080);
			out = new PrintWriter(socket.getOutputStream(), true);
			in = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
		} catch(UnknownHostException e){
			Logger.log(e);
		} catch(IOException e){
			
			Logger.log(e);
		}
	}

	/*
	 * (non-Javadoc)
	 * @see jade.core.behaviours.Behaviour#action()
	 */
	@Override
	public void action(){
		try{
			if (in.ready()){
				// is toch mijn code, dus niet zeuren
				String s = in.readLine();
				this.sent(s);
			}
		} catch(Exception e){
		}
	}

	public void sent(String msg){
		out.println(msg);
	}
}
