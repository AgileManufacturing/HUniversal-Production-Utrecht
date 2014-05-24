package HAL.factories;

import libraries.knowledgedb_client.KnowledgeDBClient;
/**
 * Generic abstract class for factories. 
 * @author Tommas Bakker
 *
 */
public abstract class Factory  {
	protected KnowledgeDBClient knowledgeDBClient;
	
	
	public Factory(KnowledgeDBClient knowledgeDBClient){
		this.knowledgeDBClient = knowledgeDBClient;
	}
}
