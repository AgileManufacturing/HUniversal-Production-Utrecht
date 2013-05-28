
package rexos.mas.productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;

import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

import rexos.libraries.blackboard_client.BlackboardClient;
import rexos.libraries.blackboard_client.GeneralMongoException;
import rexos.libraries.blackboard_client.InvalidDBNamespaceException;
import rexos.libraries.blackboard_client.InvalidJSONException;
import rexos.libraries.log.Logger;
import rexos.mas.data.Product;
import rexos.mas.data.Production;
import rexos.mas.data.ProductionEquipletMapper;
import rexos.mas.data.ProductionStep;
import rexos.mas.data.ProductionStepStatus;
import rexos.mas.equiplet_agent.EquipletDirectoryEntry;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

public class PlannerBehaviour extends OneShotBehaviour{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private ProductAgent _productAgent;

	public void plannerBehaviour(){
	}

	@Override
	public int onEnd(){
		return 0;
	}

	public static void removeEquiplet(AID aid){
		try {
			BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
			bbc.removeDocuments(aid.toString());
		} catch (UnknownHostException | GeneralMongoException | InvalidJSONException | InvalidDBNamespaceException e1) {
			// TODO Auto-generated catch block
			Logger.log(e1);
		}
	}

	@Override
	public void action(){
		try{
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
			ArrayList<ProductionStep> psa = production.getProductionSteps();
			// Retrieve the equipletmapper
			ProductionEquipletMapper pem = production
					.getProductionEquipletMapping();
			// Iterate over all the production steps
			for(ProductionStep ps : psa){
				if (ps.getStatus() == ProductionStepStatus.STATE_TODO){
					// Get the ID for the production step
					int PA_id = ps.getId();
					// Get the type of production step, aka capability
					int PA_capability = ps.getCapability();
					// Create the select query for the blackboard
					DBObject equipletCapabilityQuery = QueryBuilder
							.start("capabilities").is(PA_capability).get();
					List<DBObject> equipletDirectory = bbc
							.findDocuments(equipletCapabilityQuery);
					for(DBObject dbo : equipletDirectory){
						EquipletDirectoryEntry entry = new EquipletDirectoryEntry((BasicDBObject)dbo);
						pem.addEquipletToProductionStep(PA_id, entry.getAID());
					}
				}
			}
			// Set the production mapper in the production object
			production.setProductionEquipletMapping(pem);
			// Add the production to the product object
			product.setProduction(production);
			// Set the product object in the product agent
			this._productAgent.setProduct(product);
		} catch(Exception e){
			System.out.println("Exception planner " + e);
		}
	}
}
