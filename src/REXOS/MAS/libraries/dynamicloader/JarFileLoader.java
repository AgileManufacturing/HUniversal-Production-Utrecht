package libraries.dynamicloader;

public interface JarFileLoader {
	abstract byte[] loadJarFile() throws JarFileLoaderException;

	abstract int getBuildNumber();
}
