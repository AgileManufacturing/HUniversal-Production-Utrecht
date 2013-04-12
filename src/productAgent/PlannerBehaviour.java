package ProductAgent;

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

	public PlannerBehaviour() {
	}

	public void action() {
		try {
			_productAgent = (ProductAgent) myAgent;
			BlackboardClient bbc = new BlackboardClient("145.89.191.131",
					27017);
			bbc.setDatabase("CollectiveDb");
			bbc.setCollection("EquipletDirectory");
			
			Product product = this._productAgent.getProduct();
			Production production = product.getProduction();
			ProductionStep[] psa = production.getProductionSteps();
			
			ProductionEquipletMapper pem = production.getProductionEquipletMapping();
			
			for (ProductionStep ps : psa) {
				long PA_id = ps.getId();
				long PA_capability = ps.getCapability();
				
				DBObject equipletCapabilityQuery = QueryBuilder.start("capabilities").is(PA_capability).get();
				List<DBObject> testData = bbc.findDocuments(equipletCapabilityQuery);
				
				for(DBObject db : testData) {
					DBObject aid = (DBObject)db.get("db");
					String name = (String)aid.get("name").toString();
					pem.addEquipletToProductionStep(PA_id, new AID(name, AID.ISLOCALNAME));
				}

				System.out.println("Doing planner for productionstep " + ps.getId());
			}
			
			production.setProductionEquipletMapping(pem);
			product.setProduction(production);
			this._productAgent.setProduct(product);

		} catch (Exception e) {
			System.out.println("Exception planner " + e);
		}
	}
}
