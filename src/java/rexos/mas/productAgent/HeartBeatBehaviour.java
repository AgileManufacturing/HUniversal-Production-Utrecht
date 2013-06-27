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
package rexos.mas.productAgent;

import java.util.concurrent.TimeUnit;

import jade.core.Agent;
import jade.core.behaviours.WakerBehaviour;

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
	 * @param a
	 * @param period
	 */
	public HeartBeatBehaviour(Agent a, long period, HeartbeatReceiver hr) {
		super(a, period);
		_hr = hr;
		// TODO Auto-generated constructor stub
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

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
			
		}
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEAT_INTERVAL));
		
	}
	
	
	/*
	 *  (non-Javadoc)
	 * @see Report Heartbeat Acknowledge
	 */
	public void reportHeartBeatAck() {
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEAT_INTERVAL));
		ackReceived = true;
		heartbeatSent = false;
	}
	
	public void startHeartbeating() {
		_hr.initHeartbeat();
		heartbeatSent = true;
		ackReceived = false;
		this.reset(TimeUnit.SECONDS.toMillis(HEARTBEART_TIMEOUT_INTERVAL));
	}
	
	public void stopHeartbeating() {
		this.stop();
	}
	
	
	
	
	

}
