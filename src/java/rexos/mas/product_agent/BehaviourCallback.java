/**
 *
 * Project: Product Agent
 *
 * Package: rexos.mas.productAgent
 *
 * File: BehaviourCallback.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.product_agent;

import rexos.mas.data.BehaviourStatus;

/**
 * @author Mike
 *
 */
public interface BehaviourCallback{
	
	/**
	 * Handle Callback to Overview
	 * @param bs
	 * @param callbackArgs
	 */
	public void handleCallback(BehaviourStatus bs, Object[] callbackArgs);
	
}
