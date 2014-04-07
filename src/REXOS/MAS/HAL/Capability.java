package HAL;

import java.util.ArrayList;

import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

public abstract class Capability {
	private String name;
	public String getName() {
		return name;
	}
	
	abstract public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) 
			throws CapabilityException, FactoryException, JarFileLoaderException;
	
}
