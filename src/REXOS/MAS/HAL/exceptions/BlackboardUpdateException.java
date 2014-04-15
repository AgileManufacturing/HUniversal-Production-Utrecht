package HAL.exceptions;


/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class BlackboardUpdateException extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	/**
	 * 
	 * @param msg
	 */
	public BlackboardUpdateException(String msg, Throwable throwable){
		super(msg,throwable);
	}

}
