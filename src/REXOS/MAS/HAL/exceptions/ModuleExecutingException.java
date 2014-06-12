package HAL.exceptions;

public class ModuleExecutingException extends RuntimeException {
	private static final long serialVersionUID = 3386162951426886138L;

	public ModuleExecutingException(String message, Exception exception) {
		super(message,exception);		
	}

}
