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
package rexos.mas.productAgent;

import rexos.mas.data.BehaviourStatus;

/**
 * @author Mike
 *
 */
public interface BehaviourCallback{
	
	
	public void handleCallback(BehaviourStatus bs, Object[] callbackArgs);
	
}
