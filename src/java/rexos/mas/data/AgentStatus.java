/**
 *
 * Project: Product Agent
 *
 * Package: rexos.mas.data
 *
 * File: AgentStatus.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package rexos.mas.data;

/**
 * @author Mike
 *
 */
public enum AgentStatus{
	
	INITIALIZING(0),
	
	DONE_INITIALIZING(1),
	
	PLANNING(2),
	
	DONE_PLANNING(3),
	
	INFORMING(4),
	
	DONE_INFORMING(5),
	
	SCHEDULING(6),
	
	DONE_SCHEDULING(7),
	
	PRODUCING(8),
	
	DONE_PRODUCING(9),
	
	STARTING(10);
	
	/**
	 * @var int status
	 * The status
	 */
	private int status;
	
	/**
	 * Constructor for a status.
	 * 
	 * @param status - The status
	 */
	private AgentStatus(int status){
		this.status = status;
	}
	
	/**
	 * Function for getting the status.
	 * 
	 * @return The status as an int.
	 */
	public int getStatus(){
		return status;
	}
	
}
