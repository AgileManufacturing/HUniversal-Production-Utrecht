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

	public void plannerBehaviour(){}
	
	public int onEnd(){
		return 0;
	}
	
	public void removeEquiplet(AID aid){
		BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
		// convert the AID to the desired dbObject
		//bbc.removeDocuments(); // add the desired dbObject as param.
	}
		/*public void action() {
			ProductAgent _productAgent = null;
			final String ConversationId = _productAgent.generateCID();
			final MessageTemplate template = MessageTemplate
					.MatchConversationId(ConversationId);

			public PlannerBehaviour() {
	}*/

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
					
					
					Product product1 = this._productAgent.getProduct();
					Production production1 = product1.getProduction();
					ProductionStep[] psa1 = production1.getProductionSteps();
					
					ProductionEquipletMapper pem1 = production1.getProductionEquipletMapping();
					
					for (ProductionStep ps1 : psa1) {
						long PA_id1 = ps1.getId();
						long PA_capability1 = ps1.getCapability();
						
						DBObject equipletCapabilityQuery1 = QueryBuilder.start("capabilities").is(PA_capability1).get();
						List<DBObject> testData1 = bbc.findDocuments(equipletCapabilityQuery1);
						
						for(DBObject db : testData1) {
							String aid = (String)db.get("AID").toString();
							pem1.addEquipletToProductionStep(PA_id1, new AID(aid, true));
						}
	
					for(DBObject db : testData1) {
						DBObject aid = (DBObject)db.get("db");
						String name = (String)aid.get("name").toString();
						pem1.addEquipletToProductionStep(PA_id1, new AID(name, AID.ISLOCALNAME));
					}
	
					System.out.println("Doing planner for productionstep " + ps1.getId());
				}
				
				production1.setProductionEquipletMapping(pem1);
				product1.setProduction(production1);
				this._productAgent.setProduct(product1);
				}
	
			}catch (Exception e) {
				System.out.println("Exception planner " + e);
			}
	}
}
