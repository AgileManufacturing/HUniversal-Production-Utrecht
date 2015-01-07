package HAL.factories;

import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.JavaSoftware;
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
	protected HashMap<K , V> cache;
	
	
	public Factory(KnowledgeDBClient knowledgeDBClient) {
		this.knowledgeDBClient = knowledgeDBClient;
		dynamicClassFactory = new DynamicClassFactory<V>();
		cache = new HashMap<K,V>();
	}
	
	public void logSqlResult(LogSection logSection, String sqlQueryName, Row[] rows) {
		String message = "The SQL result from query " + sqlQueryName + ":";
		Logger.log(logSection, LogLevel.DEBUG, message, rows);
	}
	
	protected abstract JavaSoftware getJavaSoftware(K identifier);
	
	protected abstract V getConstuctorforThisFactory(Class<V> myClass, K key) throws NoSuchMethodException, SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException;
	
	public V getSomethingByIdentifier(K identifier) {
		if(cache.containsKey(identifier)) {
			return cache.get(identifier);
		}
		try {
			DynamicClassDescription description = getJavaSoftware(identifier).getDynamicClassDescription();
			Class<V> tempclass = (Class<V>) dynamicClassFactory.getClassFromDescription(description);
			V returnvalue = getConstuctorforThisFactory(tempclass, identifier);
			cache.put(identifier, returnvalue);
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
	
	public void removeItemFromCache(K identifier) {
		cache.remove(identifier);
	}
}
