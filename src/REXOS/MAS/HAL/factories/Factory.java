package HAL.factories;

import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.Row;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.LogSection;
import libraries.utillities.log.Logger;
/**
 * Generic abstract class for factories. 
 * @author Tommas Bakker
 *
 */
public abstract class Factory {
	protected KnowledgeDBClient knowledgeDBClient;
	
	
	public Factory(KnowledgeDBClient knowledgeDBClient) {
		this.knowledgeDBClient = knowledgeDBClient;
	}
	
	public void logSqlResult(LogSection logSection, String sqlQueryName, Row[] rows) {
		String message = "The SQL result from query " + sqlQueryName + ":";
		Logger.log(logSection, LogLevel.DEBUG, message, rows);
	}
}
