package productAgent;

import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;

import com.mongodb.BasicDBObject;
import com.mongodb.DBObject;
import com.mongodb.QueryBuilder;

@SuppressWarnings("serial")
public class PlannerBehaviour extends CyclicBehaviour {
	private ProductAgent _productAgent;

	public PlannerBehaviour() {
	}

	public void action() {
		_productAgent = (ProductAgent) myAgent;
		try {
			BlackboardClient bbc = new BlackboardClient("145.89.191.131",
					27017);
			bbc.setDatabase("CollectiveDb");
			bbc.setCollection("EquipletDirectory");
			BasicDBObject testObject = new BasicDBObject();
			List<DBObject> test = bbc.findDocuments(testObject);

			HashMap<AID, ArrayList<Long>> equipletCapabilities = new HashMap<AID, ArrayList<Long>>();

			Production production = this._productAgent.getProduct().getProduction();

			ProductionStep[] psa = production.getProductionSteps();

			ProductionEquipletMapper pem = production
					.getProductionEquipletMapping();
			
			for (ProductionStep ps : psa) {
				long PA_id = ps.getId();
				long PA_capability = ps.getCapability();
				
				long EQ_AID;
				List<Long> EQ_capability;
				
				DBObject equipletCapabilityQuery = QueryBuilder.start("capabilities").is(PA_capability).get();
				List<DBObject> testData = bbc.findDocuments(equipletCapabilityQuery);
				//equipletCapabilityQuery.put(key, val)
				//equipletCapabilityQuery.get

				//List<DBObject> foundEquiplets = bbc.findDocuments(equipletCapabilityQuery);
				
				AID tes123t = new AID();
				//for(DBObject bdbo : foundEquiplets) {
				//	pem.addEquipletToProductionStep(PA_id, tes123t);
				//}

				// praat met blackboardclient

			}


			// Object[] stepId = m.stepList;

			// vergelijk het stap ID met de equiplets die de stap daardwerkelijk
			// kunnen uitvoeren.
			// wanneer ze het kunnen, zet ze in een nieuwe lijst en ga naar de
			// volgende.
			// Anders ga naar de volgende.
			// if(!allEquiplets.isEmpty()){
			// System.out.println("filling 'desiredEquiplets' dictionary");
			// while(allEquiplets.elements().nextElement() != null){
			// allEquiplets.equals(stepId);
			// desiredEquiplets.put(stepId, new AID("1", AID.ISLOCALNAME));
			// }
			// }else{
			// System.out.println("the 'desiredEquiplets' dictionary could not be filled");
			// }

			// sort those who can perform the desired step

			// store those in a separate list

			// return to negotiatorbehaviour
		} catch (Exception e) {
			System.out.println("Exception");
		}
	}
}
