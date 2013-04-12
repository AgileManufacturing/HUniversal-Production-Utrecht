package hardwareAgent;

 /**
 * Author: Thierry Gerritse
 * Class: TimeData.java 
 **/

public class TimeData {
	public long duration;

	public TimeData(long duration) {
		this.duration = duration;
	}

	public long getDuration(){
		
		return this.duration;
		
	}
	
	public void setDuration(long duration){
		
		this.duration = duration;
		
	}
}
