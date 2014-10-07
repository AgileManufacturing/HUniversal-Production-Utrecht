package MAS;

/**
 * @author Lars Veenendaal
 */

import org.junit.runner.JUnitCore;
import org.junit.runner.Result;
import org.junit.runner.notification.Failure;

public class TestRunner {
    public static void main(String[] args) {
		Result result = JUnitCore.runClasses(MASTestFinder.class);
		for (Failure failure : result.getFailures()){
			System.out.print(failure.toString());
		}
		if(result.wasSuccessful()){
			System.out.println("All MAS tests passed!\n"); 
		}
	}
}