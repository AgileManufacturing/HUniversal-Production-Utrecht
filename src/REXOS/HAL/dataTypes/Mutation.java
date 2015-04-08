package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

import org.json.JSONException;
/**
 * This method provides methods for serializing and deserializing of Mutations
 * @author Tommas Bakker
 *
 */
public class Mutation implements Serializable {
	private static final long serialVersionUID = -2882231610271021724L;
	
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
	
	public String mutationType;
	
	public Mutation() {
		// nothing to do
	}
	public Mutation(String mutationType) {
		this.mutationType = mutationType;
	}
	
	public static Mutation deSerialize(String input) {
		Mutation output = new Mutation();
		
		output.mutationType = input;
		
		return output;
	}
	public String serialize() {
		return mutationType;
	}
	
	
	/**
	 * This method will return the supported mutations for the module identified by the {@link ModuleIdentifier} using the provided {@link KnowledgeDBClient}.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
	public static ArrayList<Mutation> getSupportedMutations(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		ArrayList<Mutation> mutations = new ArrayList<Mutation>();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getSupportedMutationTypesForModuleType, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		for (Row row : rows) {
			mutations.add(new Mutation((String) row.get("mutation")));
		}
		return mutations;
	}
	
	/**
	 * This method deserializes the supported mutations and copies it to the knowledge database for the module identified by the {@link ModuleIdentifier}.
	 * @param moduleTypeIdentifier
	 * @param supportedMutations
	 * @param knowledgeDBClient
	 * @return
	 * @throws JSONException 
	 */
	public static ArrayList<Mutation> insertSupportedMutations(ModuleTypeIdentifier moduleTypeIdentifier, 
			ArrayList<Mutation> supportedMutations, KnowledgeDBClient knowledgeDBClient) throws JSONException {
		ArrayList<Mutation> mutations = new ArrayList<Mutation>();
		
		for (int i = 0; i < supportedMutations.size(); i++) {
			String mutationType = supportedMutations.get(i).mutationType;
			knowledgeDBClient.executeUpdateQuery(addSupportedMutationTypeForModuleType, 
					moduleTypeIdentifier.manufacturer, moduleTypeIdentifier.typeNumber, mutationType);
			mutations.add(new Mutation(mutationType));
		}
		return mutations;
	}
	/**
	 * This method removes the supported mutations from the knowledge database for the module identified by the {@link ModuleIdentifier} 
	 * @param moduleTypeIdentifier
	 * @param knowledgeDBClient
	 */
	public static void removeSupportedMutations(ModuleTypeIdentifier moduleTypeIdentifier, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeAllSupportedMutationTypesForModuleType, 
				moduleTypeIdentifier.manufacturer, moduleTypeIdentifier.typeNumber);
	}
}
