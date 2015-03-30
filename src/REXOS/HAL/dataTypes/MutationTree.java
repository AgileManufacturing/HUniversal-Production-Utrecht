package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class MutationTree implements Serializable{
	private static final long serialVersionUID = 7549985994511586679L;
	
	public static final String TREE_NUMBER = "treeNumber";
	public static final String MUTATIONS = "mutations";
	
	/**
	 * SQL query for selecting required mutations for a capabilityType.
	 * Input: capabilityTypeName
	 */
	private static final String getRequiredMutationsForCapabilityType = 
			"SELECT mutation, treeNumber \n" + 
			"FROM CapabilityTypeRequiredMutation \n" + 
			"WHERE capabilityType = ?;";
	/**
	 * SQL query for adding a required mutation to a capabilityType.
	 * Input: treeNumber, capabilityTypeName, mutation
	 */
	private static final String addRequiredMutationForCapabilityType = 
			"INSERT IGNORE INTO CapabilityTypeRequiredMutation \n" + 
			"(treeNumber, capabilityType, mutation) \n" +
			"VALUES(?, ?, ?);";
	/**
	 * SQL query for adding a required mutation to a capabilityType.
	 * Input: treeNumber, capabilityTypeName, mutation
	 */
	private static final String removeRequiredMutation = 
			"DELETE FROM CapabilityTypeRequiredMutation \n" 
			+ "WHERE capabilityType = ? AND \n" +
			"treeNumber = ?;";

	public String capabilityName;
	public int treeNumber;
	public ArrayList<Mutation> mutations;
	
	public MutationTree() {
		mutations = new ArrayList<Mutation>();
	}
	
	public static MutationTree deSerialize(JSONObject input, String capabilityName) throws JSONException {
		MutationTree output = new MutationTree();
		
		output.capabilityName = capabilityName;
		output.treeNumber = input.getInt(TREE_NUMBER);
		
		JSONArray mutations = input.getJSONArray(MUTATIONS);
		for (int i = 0; i < mutations.length(); i++) {
			output.mutations.add(Mutation.deSerialize(mutations.getString(i)));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(TREE_NUMBER, treeNumber);
		
		JSONArray mutations = new JSONArray();
		for (int i = 0; i < this.mutations.size(); i++) {
			mutations.put(this.mutations.get(i).mutationType);
		}
		output.put(MUTATIONS, mutations);
		
		return output;
	}

	public static ArrayList<MutationTree> getMutationTreesForCapabilityName(String capabilityName, KnowledgeDBClient knowledgeDBClient) {
		HashMap<Integer, MutationTree> output = new HashMap<Integer, MutationTree>();

		Row[] rows = knowledgeDBClient.executeSelectQuery(getRequiredMutationsForCapabilityType, capabilityName);
		for (Row row : rows) {
			int treeNumber = (Integer) row.get("treeNumber");
			String mutation = (String) row.get("mutation");

			if (output.containsKey(treeNumber) == false) {
				MutationTree mutationTree = new MutationTree();
				mutationTree.treeNumber = treeNumber;
				output.put(treeNumber, mutationTree);
			}
			output.get(treeNumber).mutations.add(new Mutation(mutation));
		}
		return new ArrayList<MutationTree>(output.values());
	}
	/**
	 * This method deserializes the required mutations and stores them in the knowledge database.
	 * 
	 * @param capabilityTypeName
	 * @param requiredMutationTrees
	 * @throws JSONException
	 */
	public void insertIntoDatabase(String capabilityName, KnowledgeDBClient knowledgeDBClient) {
		for (Mutation mutation : mutations) {
			knowledgeDBClient.executeUpdateQuery(addRequiredMutationForCapabilityType, treeNumber, capabilityName, mutation.mutationType);
		}
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeRequiredMutation, capabilityName, treeNumber);
	}


}
