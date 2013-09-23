/**
 *
 * Project: ProductAgent
 *
 * Package: rexos.mas.data
 *
 * File: BehaviourStatus.java
 *
 * Author: Mike Schaap
 *
 * Version: 1.0
 *
 */
package agents.data;

/**
 * @author Mike
 *
 */
public enum BehaviourStatus {

	RUNNING(0),
	
	COMPLETED(1),
	
	ERROR(2),
	
	WAITING(3);
	
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
	private BehaviourStatus(int status){
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
