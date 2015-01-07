package MAS.simulation.test;

/**
 * @author Lars Veenendaal
 */

import org.junit.runner.JUnitCore;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses({
	MAS.simulation.test.EquipletAvailability.class,
	MAS.simulation.test.EquipletLoad.class
})


public class AllTests {
   //    public static void main(String[] args) {
             //   JUnitCore.runClasses(new Class[] { AllTests.class });
  //      }
}