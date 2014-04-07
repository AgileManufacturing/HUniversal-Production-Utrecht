package HAL;

import java.util.ArrayList;

import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

public abstract class Capability {
	private String name;
	protected ModuleFactory moduleFactory;
	
	public String getName() {
		return name;
	}
	
	public Capability(ModuleFactory moduleFactory, String capabilityName) {
		this.moduleFactory = moduleFactory;
		this.name = capabilityName;
	}
	
	abstract public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) 
			throws CapabilityException, FactoryException, JarFileLoaderException;
	
}
