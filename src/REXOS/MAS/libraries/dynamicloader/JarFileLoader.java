package libraries.dynamicloader;

public interface JarFileLoader {
	abstract byte[] loadJarFile(DynamicClassDescription description) throws JarFileLoaderException;

	abstract long getBuildNumber(DynamicClassDescription description) throws JarFileLoaderException;
}
