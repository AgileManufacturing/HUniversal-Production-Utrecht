/**
 *
 * Project: Product Agent
 *
 * Package: productAgent
 *
 * File: HeartBeartBehaviour.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package agents.product_agent.behaviours;

import java.util.concurrent.TimeUnit;

import jade.core.Agent;
import jade.core.behaviours.WakerBehaviour;

import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;

/**
 * @author Mike
 *
 */
public class HeartBeatBehaviour extends WakerBehaviour {

	private HeartbeatReceiver _hr;
	private boolean heartbeatSent = false;
	private boolean ackReceived = false;
	
	private long HEARTBEAT_INTERVAL =  5;
	private long HEARTBEART_TIMEOUT_INTERVAL = 15;
	
	
	/**
	 * Start HeartBeat over a given period for a specific Agent
	 * @param a
	 * @param period
	 */
	public HeartBeatBehaviour(Agent a, long period, HeartbeatReceiver hr) {
		super(a, period);
		_hr = hr;
		// TODO Auto-generated constructor stub
	}

	/**
	 * Serialized
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * When heartbeat is still active
	 */
	@Override
	protected void onWake() {
		try{
		if(heartbeatSent && ackReceived) {
			heartbeatSent = false;
			ackReceived = false;
		} 
		else if(heartbeatSent && !ackReceived) {
			_hr.heartbeatTimeout();
			heartbeatSent = false;
		}
		else {
			_hr.initHeartbeat();
			heartbeatSent = true;
		}
		} catch(Exception e) {
			Logger.log(LogLevel.ERROR, "gotta catch 'em all!", e);
		}
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEAT_INTERVAL));
		
	}
	
	
	/*
	 *  (non-Javadoc)
	 * @see Report Heartbeat Acknowledge
	 */
	/**
	 * Report that the heartbeat is acknowledged
	 */
	public void reportHeartBeatAck() {
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEAT_INTERVAL));
		ackReceived = true;
		heartbeatSent = false;
	}
	
	/**
	 * Starts the heartbeat
	 */
	public void startHeartbeating() {
		_hr.initHeartbeat();
		heartbeatSent = true;
		ackReceived = false;
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEART_TIMEOUT_INTERVAL));
	}
	
	/**
	 * Stops the heartbeat
	 */
	public void stopHeartbeating() {
		this.stop();
	}
	
}
