package HAL.factories;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.JavaSoftware;
import HAL.libraries.dynamicloader.DynamicClassDescription;
import HAL.libraries.dynamicloader.DynamicClassFactory;
import HAL.libraries.dynamicloader.InstantiateClassException;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;
/**
 * Generic abstract class for factories. 
 * @author Tommas Bakker
 * @author Niek Arends
 *
 */

public abstract class Factory <K ,V> {
	protected KnowledgeDBClient knowledgeDBClient;
	private DynamicClassFactory<?> dynamicClassFactory;
	private HashMap<K , V> instancesCache;
	
	public Factory(KnowledgeDBClient knowledgeDBClient) {
		this.knowledgeDBClient = knowledgeDBClient;
		dynamicClassFactory = new DynamicClassFactory<V>();
		instancesCache = new HashMap<K,V>();
	}
	
	public void logSqlResult(LogSection logSection, String sqlQueryName, Row[] rows) {
		String message = "The SQL result from query " + sqlQueryName + ":";
		Logger.log(logSection, LogLevel.DEBUG, message, rows);
	}
	
	protected abstract JavaSoftware getJavaSoftware(K identifier);
	protected abstract V getConstuctorforThisFactory(Class<V> myClass, K key) throws NoSuchMethodException, SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException;
	public V getItemForIdentifier(K identifier) {
		if(instancesCache.containsKey(identifier)) {
			return instancesCache.get(identifier);
		}
		try {
			DynamicClassDescription description = getJavaSoftware(identifier).getDynamicClassDescription();
			@SuppressWarnings("unchecked")
			Class<V> tempclass = (Class<V>) dynamicClassFactory.getClassFromDescription(description);
			V returnvalue = getConstuctorforThisFactory(tempclass, identifier);
			instancesCache.put(identifier, returnvalue);
			return returnvalue;
		} catch (IllegalArgumentException | NoSuchMethodException | SecurityException | 
				InstantiateClassException | InstantiationException | IllegalAccessException | InvocationTargetException ex) {
			Logger.log(LogSection.HAL_FACTORIES, LogLevel.CRITICAL, "well, we are fucked", ex);
			return null;
		} catch (JarFileLoaderException ex) {
			Logger.log(LogSection.HAL_FACTORIES, LogLevel.CRITICAL, "Unable to load the jarFile for this module", ex);
			return null;
		}
	}
	
	protected abstract ArrayList<K> getKeysToKeepInCache();
	
	public void checkCache() {
		// avoid invalidating the iterator by using two stage removal
		ArrayList<K> keysToKeep = getKeysToKeepInCache();
		ArrayList<K> keysToRemove = new ArrayList<K>();
		for (K key : instancesCache.keySet()) {
			if(keysToKeep.contains(key) == false) {
				keysToRemove.add(key);
			}
		}
		
		for (K key : keysToRemove) {
			instancesCache.remove(key);
		}
	}
}
