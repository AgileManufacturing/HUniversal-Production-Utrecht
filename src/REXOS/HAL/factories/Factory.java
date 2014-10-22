package HAL.factories;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.JavaSoftware;
import HAL.Module;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
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
	private HashMap<K , V> cache;
	private HardwareAbstractionLayer hal;
	
	
	public Factory(KnowledgeDBClient knowledgeDBClient, HardwareAbstractionLayer hal) {
		this.knowledgeDBClient = knowledgeDBClient;
		this.hal = hal;
		dynamicClassFactory = new DynamicClassFactory<V>();
		cache = new HashMap<K,V>();
	}
	
	public void logSqlResult(LogSection logSection, String sqlQueryName, Row[] rows) {
		String message = "The SQL result from query " + sqlQueryName + ":";
		Logger.log(logSection, LogLevel.DEBUG, message, rows);
	}
	
	protected abstract JavaSoftware getJavaSoftware(K identifier);
	
	protected abstract V getConstuctorforThisFactory(Class<V> myClass, K key) throws NoSuchMethodException, SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException;
	
	public V getSomethingByIdentifier(K identifier) throws FactoryException, JarFileLoaderException {
		DynamicClassDescription description = getJavaSoftware(identifier).getDynamicClassDescription();
		System.out.println("Waar ga ik dood?");
		for(K cacheidentifier : cache.keySet()) {
			if(cacheidentifier.equals(cache) == true) {
				System.out.println("is het hier?");
				return cache.get(identifier);
			}
		}
		try {
			System.out.println("Aapkip?");
			Class<V> tempclass = (Class<V>) dynamicClassFactory.getClassFromDescription(description);
			V returnvalue = getConstuctorforThisFactory(tempclass, identifier);
			System.out.println("Misschien hier?");

			return returnvalue;
		} catch (IllegalArgumentException | NoSuchMethodException | SecurityException | InstantiateClassException | InstantiationException | IllegalAccessException | InvocationTargetException ex) {
			throw new FactoryException("well, we are fucked", ex);
		}
	}
}
