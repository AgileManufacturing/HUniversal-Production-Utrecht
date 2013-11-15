package libraries.schedule.data_classes;

public class ScheduleException extends Exception{

	/**
	 * @var long serialVersionUID
	 * 		The serialization UID of this class
	 */
	private static final long serialVersionUID = 6623350352232864851L;
	
	public ScheduleException(String msg){
		super(msg);
	}

}
