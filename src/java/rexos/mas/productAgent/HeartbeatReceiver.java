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
	
	public void initHeartbeat();
	
	public void heartbeatTimeout();
}
