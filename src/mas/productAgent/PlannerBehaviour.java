package productAgent;

import java.util.Dictionary;
import java.util.Hashtable;

import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionEquipletMapper;
import newDataClasses.ProductionStep;
//import main.MainAgent;
import jade.core.AID;
import jade.core.behaviours.CyclicBehaviour;

@SuppressWarnings("serial")
public class PlannerBehaviour extends CyclicBehaviour{
	private ProductAgent _productAgent;

	public PlannerBehaviour(ProductAgent pa) {
		this._productAgent = pa;
	}
	
	public void action() {
		// check all available equiplets	
		//MainAgent m = new MainAgent();
		//Dictionary<AID, String> allEquiplets = m.allEquiplets;
		@SuppressWarnings({ "unchecked", "rawtypes" })
		Dictionary<Object, AID> desiredEquiplets = new Hashtable(); // contains the current step + the agent ID

		
		
		Product p = this._productAgent.getProduct();
		
		Production production = p.getProduction();
		
		ProductionStep[] psa = production.getProductionSteps();
		
		for(ProductionStep ps : psa) {
			long id = ps.getId();
		}
		
		ProductionEquipletMapper pem = production.getProductionEquipletMapping();
	/*	if(!allEquiplets.isEmpty()){
			System.out.println("filling 'desiredEquiplets' dictionary");
			while(allEquiplets.elements().nextElement() != null){
				allEquiplets.equals(stepId);
				desiredEquiplets.put(stepId, new AID("1", AID.ISLOCALNAME));
			}
		}else{
			System.out.println("the 'desiredEquiplets' dictionary could not be filled");
		}
		*/
		//if(!allEquiplets.isEmpty()){
		//	System.out.println("filling 'desiredEquiplets' dictionary");
		//	while(allEquiplets.elements().nextElement() != null){
		//		allEquiplets.equals(stepId);
		//		desiredEquiplets.put(stepId, new AID("1", AID.ISLOCALNAME));
		//	}
		//}else{
		//	System.out.println("the 'desiredEquiplets' dictionary could not be filled");
		//}
		
		
		
		// sort those who can perform the desired step
		
		// store those in a separate list
	
		// return to negotiatorbehaviour
	}
}
