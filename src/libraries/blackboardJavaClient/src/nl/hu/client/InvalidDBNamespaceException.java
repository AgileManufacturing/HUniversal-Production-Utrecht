package nl.hu.client;

public class InvalidDBNamespaceException extends Exception {

	/**
	 * @var long serialVersionUID
	 * SerialUID for this class.
	 */
	private static final long serialVersionUID = 6479137977336676219L;
	
	/**
	 * Constructs an InvalidDBNamespaceException object with the specified message.
	 * 
	 * @param message Message describing the contents of this exception.
	 */
	public InvalidDBNamespaceException(String message) {
		super(message);
	}
}
