package hardwareAgent;

 /**
 * Author: Thierry Gerritse
 * Class: TimeData.java 
 **/

public class TimeData {
	/**
	 * @var long duration
	 * the length of duration 
	 */
	public long duration;
	
	/**
	 * 
	 * @param duration
	 */
	public TimeData(long duration) {
		this.duration = duration;
	}
	/**
	 * 
	 * @return duration
	 */
	public long getDuration(){
		
		return this.duration;
		
	}
	/**
	 * 
	 * @param duration
	 */
	public void setDuration(long duration){
		
		this.duration = duration;
		
	}
}
