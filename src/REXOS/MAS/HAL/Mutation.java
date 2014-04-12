package HAL;

import java.util.ArrayList;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonPrimitive;

public class Mutation {
	private static final String getSupportedMutationTypesForModuleType =
			"SELECT mutation \n" + 
			"FROM SupportedMutation \n" + 
			"WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ?;";
	private static final String addSupportedMutationTypeForModuleType =
			"INSERT INTO SupportedMutation \n" + 
			"(manufacturer, typeNumber, mutation) \n" + 
			"VALUES(?, ?, ?);";
	private static final String removeAllSupportedMutationTypesForModuleType =
			"DELETE FROM SupportedMutation \n" + 
			"WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ?;";
	
	private String mutationType;
	public String getMutationType() {
		return mutationType;
	}
	
	public Mutation(String mutationType) {
		this.mutationType = mutationType;
	}
	
	public static ArrayList<Mutation> getSupportedMutations(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		try{
			ArrayList<Mutation> mutations = new ArrayList<Mutation>();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedMutationTypesForModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			for (Row row : rows) {
				mutations.add(new Mutation((String) row.get("mutation")));
			}
			return mutations;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::Mutation::getSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static JsonArray serializeAllSupportedMutations(ModuleIdentifier moduleIdentifier) {
		try {
			return serializeAllSupportedMutations(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::serializeAllSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static JsonArray serializeAllSupportedMutations(ModuleIdentifier moduleIdentifier, 
			KnowledgeDBClient knowledgeDBClient) {
		JsonArray supportedMutationEntries = new JsonArray();
		ArrayList<Mutation> mutations = getSupportedMutations(moduleIdentifier, knowledgeDBClient);
		for (Mutation mutation : mutations) {
			supportedMutationEntries.add(new JsonPrimitive(mutation.getMutationType()));
		}
		return supportedMutationEntries;
	}
	public static ArrayList<Mutation> insertSupportedMutations(ModuleIdentifier moduleIdentifier, JsonArray supportedMutationEntries) {
		try{
			return insertSupportedMutations(moduleIdentifier, supportedMutationEntries, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::insertSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static ArrayList<Mutation> insertSupportedMutations(ModuleIdentifier moduleIdentifier, 
			JsonArray supportedMutationEntries, KnowledgeDBClient knowledgeDBClient) {
		try {
			ArrayList<Mutation> mutations = new ArrayList<Mutation>();
			
			for (JsonElement supportedMutationEntryElement : supportedMutationEntries) {
				String mutationType = supportedMutationEntryElement.getAsString();
				knowledgeDBClient.executeUpdateQuery(addSupportedMutationTypeForModuleType, 
						moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber(), mutationType);
				mutations.add(new Mutation(mutationType));
			}
			return mutations;
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::insertSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static void removeSupportedMutations(ModuleIdentifier moduleIdentifier) {
		try {
			removeSupportedMutations(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::removeSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
	public static void removeSupportedMutations(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		try {
			knowledgeDBClient.executeUpdateQuery(removeAllSupportedMutationTypesForModuleType, 
					moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::removeSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
}
