package libraries.dynamicloader;

public class JarFileLoaderException extends Exception {
	public JarFileLoaderException(String message, Throwable cause) {
		super(message, cause);
	}

	public JarFileLoaderException(String message) {
		super(message);
	}
}
