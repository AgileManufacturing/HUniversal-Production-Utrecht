package HAL.libraries.dynamicloader;

public class JarFileLoaderException extends Exception {
	private static final long serialVersionUID = -156540560179409185L;

	public JarFileLoaderException(String message, Throwable cause) {
		super(message, cause);
	}

	public JarFileLoaderException(String message) {
		super(message);
	}
}
