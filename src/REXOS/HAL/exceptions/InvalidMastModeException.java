package HAL.exceptions;

public class InvalidMastModeException extends Exception {
	private static final long serialVersionUID = 384534193218517588L;
	
	public InvalidMastModeException(String message) {
		super(message);
	}
	public InvalidMastModeException(String message, Throwable throwable) {
		super(message, throwable);
	}
}
