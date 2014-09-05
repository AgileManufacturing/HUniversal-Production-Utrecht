package util.configuration;

public enum ConfigurationFiles {
	
	MONGO_DB_PROPERTIES("mongo_db.properties"),
	KNOWLEDGE_DB_PROPERTIES("knowledge_db.properties"),
	EQUIPLET_DB_PROPERTIES("equiplet_db.properties");

	String fileName;

	ConfigurationFiles(String fileName){
		this.fileName = fileName;
	}
	
	public String getFileName(){
		return this.fileName;
	}

}