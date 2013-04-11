package productAgent;

import jade.core.AID;
import jade.core.behaviours.OneShotBehaviour;
import java.util.List;
import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;
import jade.lang.acl.ACLMessage;
import jade.lang.acl.MessageTemplate;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

@SuppressWarnings("serial")
public class PlannerBehaviour extends OneShotBehaviour {
	private AID _aid;

	private ProductAgent _productAgent;

	public void plannerBehaviour(){}
	
	public int onEnd(){
		return 0;
	}	
		public void action() {
			ProductAgent _productAgent = null;
			final String ConversationId = _productAgent.generateCID();
			final MessageTemplate template = MessageTemplate
					.MatchConversationId(ConversationId);
			_productAgent = (ProductAgent) myAgent;
			try {
				ACLMessage message = new ACLMessage(ACLMessage.REQUEST);
				message.setConversationId(ConversationId);
				message.addReceiver(_aid);
				message.setOntology("planningBehaviour");
				_productAgent.send(message);
				
				
				BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
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
						String aid = (String)db.get("AID").toString();
						pem.addEquipletToProductionStep(PA_id, new AID(aid, true));
					}
				}
				
				production.setProductionEquipletMapping(pem);
				product.setProduction(production);
				this._productAgent.setProduct(product);
	
			} catch (Exception e) {
				System.out.println("Exception");
			}		
		}
	}

