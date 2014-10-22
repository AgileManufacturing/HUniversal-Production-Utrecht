package HAL.factories;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;
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
