/**
 * @file rexos/mas/productAgent/SocketBehaviour.java
 * @brief Behaviour in which the product agent connects to the socket server to
 *        communicate with the web interface.
 * @date Created: 17-04-2013
 * 
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.productAgent;

import jade.core.Agent;
import jade.core.behaviours.WakerBehaviour;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.concurrent.TimeUnit;

import com.google.gson.Gson;

import rexos.mas.data.Callback;
import rexos.mas.data.GUIMessage;

public class SocketBehaviour extends WakerBehaviour implements
		HeartbeatReceiver {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Socket socket;
	private PrintWriter outputStream = null;
	private BufferedReader inputStream = null;
	private boolean isConnected = false;
	private HeartBeatBehaviour _hbb;
	private Gson _gsonParser;

	private Callback _callback;

	/**
	 * Constructs socket behavior
	 * @param a
	 * @param callback
	 */
	public SocketBehaviour(Agent a, Callback callback) {
		super(a, 4500);
		try {
			_callback = callback;
			_gsonParser = new Gson();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Sets the heartbeat
	 * @param hbb
	 */
	public void setHeartBeatBehaviour(HeartBeatBehaviour hbb) {
		this._hbb = hbb;
	}

	/**
	 * Keeps track of the heartbeat
	 */
	@Override
	protected void onWake() {
		try {
			if (!isConnected) {
				connect();
			} else {
				if (inputStream.ready()) {
					String s = inputStream.readLine();
					if (s.equals("beat")) {
						_hbb.reportHeartBeatAck();
					}
				}
			}
		} catch (Exception e) {
			isConnected = false;
		}
		this.reset(4500);
	}

	/**
	 * Write message through the stream
	 * @param msg
	 */
	public void write(String msg) {
		try {
			if (isConnected) {
				outputStream.println(msg);
			}
		} catch (Exception e) {
			isConnected = false;
		}
	}

	/**
	 * Write message through the stream
	 * @param error
	 * @param msg
	 * @param payload
	 */
	public void write(boolean error, String msg, String payload) {
		try {
			if (isConnected) {
				GUIMessage guiMsg = new GUIMessage();
				guiMsg.setError(error);
				guiMsg.setMessage(msg);
				guiMsg.setPayload(payload);
				write(guiMsg);
			}
		} catch (Exception e) {
			isConnected = false;
		}
	}

	/**
	 * Write message through the stream
	 * @param guiMsg
	 */
	public void write(GUIMessage guiMsg) {
		try {
			if (isConnected) {
				String output = "";
				output = _gsonParser.toJson(guiMsg, GUIMessage.class);
				outputStream.println(output);
			}
		} catch (Exception e) {
			isConnected = false;
		}
	}

	/**
	 * Connect to socket
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void connect() throws UnknownHostException, IOException {
		socket = new Socket();
		socket.connect(
				new InetSocketAddress(_callback.getHost(), _callback.getPort()),500);
		outputStream = new PrintWriter(socket.getOutputStream(), true);
		inputStream = new BufferedReader(new InputStreamReader(
				socket.getInputStream()));
		isConnected = true;
		_hbb.startHeartbeating();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see newDataClasses.HeartbeatReceiver#initHeartbeat()
	 */
	/**
	 * Initialize heartbeat
	 */
	@Override
	public void initHeartbeat() {
		try {
			if (isConnected) {
				outputStream.println("heart");
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see newDataClasses.HeartbeatReceiver#heartbeatTimeout()
	 */
	/**
	 * Defines what to do when heartbeat times out
	 */
	@Override
	public void heartbeatTimeout() {
		try {
			this.isConnected = false;
			this.resetConnection();
		} catch (Exception e) {
			// System.out.println("Error when resetting the connection");
		}
	}

	/**
	 * Resets socket connection
	 * @throws IOException
	 */
	public void resetConnection() throws IOException {
		this.socket.close();
		//this.connect();
	}

	/**
	 * Stops socket and heartbeat.
	 */
	public void stop() {
		this._hbb.stopHeartbeating();
	}
}
