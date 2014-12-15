package MAS.simulation.mas.product.legacy;

import jade.core.AID;
import jade.lang.acl.ACLMessage;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.json.JSONException;

import MAS.product.ProductStep;
import MAS.product.ProductionStep;
import MAS.util.Ontology;
import MAS.util.Parser;
import MAS.util.Position;
import MAS.util.Tick;
import MAS.util.Tuple;

public class Scheduling {

	public void schedule(double time, Position position) {
		// equiplets = Map < Equiplet, < List<Service able to perform>, load of equiplet, first possibility >
		ArrayList<Tuple<AID, List<String>, Double, Double>> equiplets = new ArrayList<>();

		ArrayList<ProductStep> productSteps = new ArrayList<>();

		double[][] matrix = generateMatrix(equiplets, productSteps);
		schedule(time, position, equiplets, productSteps, matrix);

	}

	/**
	 * Generates the scheduleMatrix for all productsteps & equiplets. For more information see
	 * 
	 * @ref to paper 'Multiagent-based agile manufacturing: from user requirements to product' - Leo van Moergestel section 3.2
	 * 
	 * @param equiplets
	 *            Map < Equiplet, < List<Service able to perform>, load of equiplet>
	 * @param productSteps
	 *            list of product steps of the product
	 * @return the generated scheduleMatrix
	 */
	private double[][] generateMatrix(ArrayList<Tuple<AID, List<String>, Double, Double>> equiplets, ArrayList<ProductStep> productSteps) {
		double[][] matrix = new double[equiplets.size()][productSteps.size()];
		// initialize matrix
		for (int r = 0; r < matrix.length; r++) {
			for (int c = 0; c < matrix.length; c++) {
				matrix[r][c] = 1.0;
			}
		}

		// iterate over the equiplets, the rows of the matrix
		for (int row = 0; row < equiplets.size(); row++) {
			List<String> capableServices = equiplets.get(row).second;
			double load = equiplets.get(row).third;

			// always set sequenceLength to 0 and firstInsequence to -1 when doing a new row.
			// the sequenceLength is the number of the product steps that can be consecutive performed by an equiplet
			int sequenceLength = 0;

			// the first product step in the sequence that an equiplet can perform
			int firstInSequence = -1;

			// iterate over the product steps, the columns of the matrix
			for (int column = 0; column < productSteps.size(); column++) {
				ProductStep productStep = productSteps.get(column);

				// if equiplet can perform product step
				// TODO match criteria if needed
				if (capableServices.contains(productStep.getService())) {
					// set the first item in the sequence.
					if (firstInSequence < 0) {
						firstInSequence = column;
					}
					sequenceLength++;

					if (column == productSteps.size() - 1) { // end of row
						// set value of sequence
						for (int c = firstInSequence; c < matrix[row].length; c++) {
							int value = sequenceLength - 1;
							matrix[row][c] = value;
						}
					}
				} else if (sequenceLength > 0) {
					// end of sequence, set value of sequence
					for (int c = firstInSequence; c < matrix[row].length; c++) {
						int value = sequenceLength - 1;
						matrix[row][c] = value;
					}
					sequenceLength = 0;
					firstInSequence = -1;
				}

				// value might have changed since we added sequence multiplier
				// TODO: perform supermagic calculation of currentValue * load here
				for (int c = firstInSequence; c < matrix[row].length; c++) {
					int value = sequenceLength - 1;
					matrix[row][c] = value;
				}
				matrix[row][column] = matrix[row][column] * load;
			}
		}

		return matrix;
	}


	private void schedule(double time, Position position, ArrayList<Tuple<AID, List<String>, Double, Double>> equiplets, ArrayList<ProductStep> productSteps, double[][] matrix) {
		ArrayList<ProductionStep> productionPath = new ArrayList<ProductionStep>();

		// for each product step
		for (int column = 0; column < matrix[0].length; column++) {
			// the index with the highest score
			int highestEquipletScoreIndex = -1;

			ProductStep productStep = productSteps.get(column);

			// look for the equiplet with the highest score for performing the product step
			for (int row = 0; row < matrix.length; row++) {
				highestEquipletScoreIndex = matrix[row][column] > matrix[highestEquipletScoreIndex][column] ? row : highestEquipletScoreIndex;
			}

			if (highestEquipletScoreIndex < 0) {
				System.out.println("No suitable equiplet found for this step! Scheduling has gone wrong.. Reschedule?");
				// return false;
			}

			// Can we assume that all productSteps are ordered? What about parallel steps?

			// Get first free time
			double firstPossibility = equiplets.get(highestEquipletScoreIndex).fourth;
			
			// adjusted scheduling from project dec 2013, making consider time of previous product steps and changing time slot mechanism
			
			// TODO construct production path
			Math.max(time, firstPossibility);
		}
	}
	
	private void sendScheduleMessages(ArrayList<ProductionStep> productionPath, Tick deadline) {
		// group the production path by equiplets, so to send multiple product steps schedule requests at once to one equiplet. 
		HashMap<AID, ArrayList<ProductionStep>> sendMap = new HashMap<>();
		
		for(ProductionStep productionStep : productionPath) {
			AID equipletAid = productionStep.getEquiplet();
			
			if(!sendMap.keySet().contains(equipletAid)){
				//item doesn't exists, insert the productStep schedule
				sendMap.put(equipletAid, new ArrayList<ProductionStep>());
			}
			sendMap.get(equipletAid).add(productionStep);
		}
				
		for (AID equipletAid : sendMap.keySet()) {
			try {
				ArrayList<ProductionStep> steps = sendMap.get(equipletAid);
				String content = Parser.parseScheduleRequest(steps, deadline);
				
				// Ask the equiplet to schedule the service
				ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
				message.addReceiver(equipletAid);
				message.setOntology(Ontology.GRID_ONTOLOGY);
				message.setConversationId(Ontology.CONVERSATION_SCHEDULE);
				message.setReplyWith(Ontology.CONVERSATION_SCHEDULE + System.currentTimeMillis());
				message.setContent(content);
				// send(message);
				
			} catch (JSONException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		
			// wait on confirmations
			// check if all succeeded
		}
	}
}
