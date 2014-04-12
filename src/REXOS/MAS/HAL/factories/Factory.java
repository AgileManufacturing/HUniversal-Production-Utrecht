package HAL.factories;

import libraries.knowledgedb_client.KnowledgeDBClient;

public class Factory  {
	protected KnowledgeDBClient knowledgeDBClient;
	
	
	public Factory(KnowledgeDBClient knowledgeDBClient){
		this.knowledgeDBClient = knowledgeDBClient;
	}
}
