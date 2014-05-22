package HAL;

import java.util.ArrayList;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonPrimitive;
/**
 * This method provides methods for serializing and deserializing of Mutations
 * @author Tommas Bakker
 *
 */
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
	/**
	 * This method will return the supported mutations for the module identified by the {@link ModuleIdentifier} using the provided {@link KnowledgeDBClient}.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
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
	/**
	 * This method serializes the supported mutations from the knowledge database for the module identified by the {@link ModuleIdentifier}. This method does NOT remove the supported mutations from the knowledge database.
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param moduleIdentifier
	 * @return
	 */
	public static JsonArray serializeAllSupportedMutations(ModuleIdentifier moduleIdentifier) {
		try {
			return serializeAllSupportedMutations(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::serializeAllSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	/**
	 * This method serializes the supported mutations from the knowledge database for the module identified by the {@link ModuleIdentifier}. This method does NOT remove the supported mutations from the knowledge database.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
	public static JsonArray serializeAllSupportedMutations(ModuleIdentifier moduleIdentifier, 
			KnowledgeDBClient knowledgeDBClient) {
		JsonArray supportedMutationEntries = new JsonArray();
		ArrayList<Mutation> mutations = getSupportedMutations(moduleIdentifier, knowledgeDBClient);
		for (Mutation mutation : mutations) {
			supportedMutationEntries.add(new JsonPrimitive(mutation.getMutationType()));
		}
		return supportedMutationEntries;
	}
	/**
	 * This method deserializes the supported mutations and copies it to the knowledge database for the module identified by the {@link ModuleIdentifier}.
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param moduleIdentifier
	 * @param supportedMutationEntries
	 * @return
	 */
	public static ArrayList<Mutation> insertSupportedMutations(ModuleIdentifier moduleIdentifier, JsonArray supportedMutationEntries) {
		try{
			return insertSupportedMutations(moduleIdentifier, supportedMutationEntries, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::insertSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
			return null;
		}
	}
	/**
	 * This method deserializes the supported mutations and copies it to the knowledge database for the module identified by the {@link ModuleIdentifier}.
	 * @param moduleIdentifier
	 * @param supportedMutationEntries
	 * @param knowledgeDBClient
	 * @return
	 */
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
	/**
	 * This method removes the supported mutations from the knowledge database for the module identified by the {@link ModuleIdentifier} 
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param moduleIdentifier
	 */
	public static void removeSupportedMutations(ModuleIdentifier moduleIdentifier) {
		try {
			removeSupportedMutations(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::Mutation::removeSupportedMutations(): Error occured which is considered to be impossible " + ex);
			ex.printStackTrace();
		}
	}
	/**
	 * This method removes the supported mutations from the knowledge database for the module identified by the {@link ModuleIdentifier} 
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 */
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
