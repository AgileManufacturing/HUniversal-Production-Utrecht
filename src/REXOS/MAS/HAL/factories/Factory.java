package HAL.factories;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.JarFileLoader;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

public class Factory implements JarFileLoader {
	// SQL queries
	private static final String getJarFileForDescription = 
			"SELECT jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ?;";
	private static final String getBuildNumberForDescription = 
			"SELECT buildNumber \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ?;";
	
	protected KnowledgeDBClient knowledgeDBClient;
	
	@Override
	public byte[] loadJarFile(DynamicClassDescription description) throws JarFileLoaderException{
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJarFileForDescription, description.getId());
			for (Row row : rows) {
				byte[] jarFile = (byte[]) row.get("jarFile");
				System.out.println("file size: " + jarFile.length);
				return jarFile;
			}
			throw new JarFileLoaderException("HAL::Factory::loadJarFile(): Unable to retrieve the software");
		} catch (KnowledgeException | KeyNotFoundException ex) {
			throw new JarFileLoaderException("HAL::Factory::loadJarFile(): Error occured which is considered to be impossible", ex);
		}
	}

	@Override
	public long getBuildNumber(DynamicClassDescription description) throws JarFileLoaderException {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getBuildNumberForDescription, description.getId());
			for (Row row : rows) {
				long buildNumber = (Integer) row.get("buildNumber");
				return buildNumber;
			}
			throw new JarFileLoaderException("HAL::Factory::getBuildNumber(): Unable to retrieve the software");
		} catch (KnowledgeException | KeyNotFoundException ex) {
			throw new JarFileLoaderException("HAL::Factory::loadJarFile(): Error occured which is considered to be impossible", ex);
		}
	}
	
	public Factory(KnowledgeDBClient knowledgeDBClient){
		this.knowledgeDBClient = knowledgeDBClient;
	}
}
