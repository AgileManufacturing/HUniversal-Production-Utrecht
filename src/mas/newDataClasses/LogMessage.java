/**
 * 
 */
package newDataClasses;


/**
 * @author Theodoor
 *
 */
public class LogMessage {
	/**
	 * @param id
	 * @param time
	 * @param message
	 * @param state
	 */
	public LogMessage(String id, String time, String message, String state) {
		super();
		this.id = id;
		this.time = time;
		this.message = message;
		this.state = state;
	}
	private String id;
	/**
	 * @return the id
	 */
	public String getId() {
		return id;
	}
	/**
	 * @param id the id to set
	 */
	public void setId(String id) {
		this.id = id;
	}
	/**
	 * @return the time
	 */
	public String getTime() {
		return time;
	}
	/**
	 * @param time the time to set
	 */
	public void setTime(String time) {
		this.time = time;
	}
	/**
	 * @return the message
	 */
	public String getMessage() {
		return message;
	}
	/**
	 * @param message the message to set
	 */
	public void setMessage(String message) {
		this.message = message;
	}
	/**
	 * @return the state
	 */
	public String getState() {
		return state;
	}
	/**
	 * @param state the state to set
	 */
	public void setState(String state) {
		this.state = state;
	}
	private String time;
	private String message;
	private String state;
}
