package productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;

import java.util.List;

import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;

import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

@SuppressWarnings("serial")
public class PlannerBehaviour extends OneShotBehaviour {
	private ProductAgent _productAgent;

	public void plannerBehaviour() {
	}

	public int onEnd() {
		return 0;
	}

	public void removeEquiplet(AID aid) {
		BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
		// convert the AID to the desired dbObject
		// bbc.removeDocuments(); // add the desired dbObject as param.
	}

	public void action() {
		try {
			// Get the root Agent
			_productAgent = (ProductAgent) myAgent;
			// Create the blackboardclient to connect to a specific IP & port
			BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
			// Select the database
			bbc.setDatabase("CollectiveDb");
			// Select the collection
			bbc.setCollection("EquipletDirectory");
			// Get the product object
			Product product = this._productAgent.getProduct();
			// Get the production object
			Production production = product.getProduction();
			// Retrieve the productionstep array
			ProductionStep[] psa = production.getProductionSteps();
			// Retrieve the equipletmapper
			ProductionEquipletMapper pem = production
					.getProductionEquipletMapping();
			// Iterate over all the production steps
			for (ProductionStep ps : psa) {
				// Get the ID for the production step
				int PA_id = ps.getId();
				// Get the type of production step, aka capability
				long PA_capability = ps.getCapability();
				// Create the select query for the blackboard
				DBObject equipletCapabilityQuery = QueryBuilder
						.start("capabilities").is(PA_capability).get();
				List<DBObject> equipletDirectory = bbc
						.findDocuments(equipletCapabilityQuery);

				for (DBObject dbo : equipletDirectory) {
					DBObject aid = (DBObject) dbo.get("db");
					String name = (String) aid.get("name").toString();
					pem.addEquipletToProductionStep(PA_id, new AID(name,
							AID.ISLOCALNAME));
				}

				System.out.println("Doing planner for productionstep " + PA_id);
			}
			// Set the production mapper in the production object
			production.setProductionEquipletMapping(pem);
			// Add the production to the product object
			product.setProduction(production);
			// Set the product object in the product agent
			this._productAgent.setProduct(product);

		} catch (Exception e) {
			System.out.println("Exception planner " + e);
		}
	}
}
