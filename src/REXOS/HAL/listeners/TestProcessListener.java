package HAL.listeners;

import java.util.ArrayList;

import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public interface TestProcessListener {
	void onTestFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareSteps);
	void onTestFailed(ProductStep productStep);
}
