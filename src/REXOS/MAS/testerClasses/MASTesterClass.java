package MAS.testerClasses;

import MAS.equiplet.EquipletAgent;

public class MASTesterClass {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		//Create new EquipletAgent and initalize it
		EquipletAgent testEQ = new EquipletAgent();
		testEQ.setup();
		
		//Add some modules
		//testEQ.run();
		
		//Test the reconfig system
//		System.out.printf("EQ stats: %s", testEQ.getAgentState());
//		System.out.printf("EQ All modules: %s", testEQ.getAllModules());
//		System.out.printf("EQ Name: %s", testEQ.getName());
//		testEQ.reconfigureEquiplet(testEQ.getAllModules());
	}

}
