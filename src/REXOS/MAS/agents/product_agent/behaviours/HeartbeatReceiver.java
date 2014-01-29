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
package agents.product_agent.behaviours;

/**
 * @author Mike
 *
 */
public interface HeartbeatReceiver {
	
	/**
	 * Initialize heartbeat
	 */
	public void initHeartbeat();
	
	/**
	 * Defines what to do when the heartbeat times out
	 */
	public void heartbeatTimeout();
}
