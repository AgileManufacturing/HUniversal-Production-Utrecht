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
package agents.product_agent;

import agents.data.BehaviourStatus;

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
