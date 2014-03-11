package libraries.dynamicloader;

public interface JarFileLoader {
	abstract byte[] loadJarFile(DynamicClassDescription description);

	abstract long getLastModified(DynamicClassDescription description);
}
