/**
 *
 * Project: Product Agent
 *
 * Package: newDataClasses
 *
 * File: HeartbeatInterface.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.productAgent;

/**
 * @author Mike
 *
 */
public interface HeartbeatReceiver {
	
	/**
	 * initialize heartbeart
	 */
	public void initHeartbeat();
	
	/**
	 * When an timeout occurs during the heartbeat
	 */
	public void heartbeatTimeout();
}
