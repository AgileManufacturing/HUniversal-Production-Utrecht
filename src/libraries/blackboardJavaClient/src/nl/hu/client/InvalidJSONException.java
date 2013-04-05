package nl.hu.client;

import com.mongodb.util.JSONParseException;

public class InvalidJSONException extends Exception {
	/**
	 * @var long serialVersionUID
	 * Serial version ID of this class.
	 */
	private static final long serialVersionUID = -7981116871524728088L;

	/**
	 * Constructs an InvalidJSONException storing the JSONParseException as its cause.
	 * 
	 * @param ex The JSONParseException that caused this exception.
	 */
	public InvalidJSONException(JSONParseException ex) {
		super("An error has occurred while parsing the JSON string.", ex);
	}
}
