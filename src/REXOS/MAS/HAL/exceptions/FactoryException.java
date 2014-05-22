package HAL.exceptions;

public class FactoryException extends Exception {
	private static final long serialVersionUID = -6143861578812032904L;
	public FactoryException(String message) {
		super(message);
	}
	public FactoryException(String message, Throwable cause) {
		super(message, cause);
	}
}
