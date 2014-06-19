package HAL.exceptions;

public class ModuleExecutingException extends Exception {
	private static final long serialVersionUID = 3386162951426886138L;

	public ModuleExecutingException(String message, Exception exception) {
		super(message,exception);		
	}
	public ModuleExecutingException(String message) {
		super(message);		
	}

}
