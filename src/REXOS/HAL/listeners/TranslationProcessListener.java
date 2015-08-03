package HAL.listeners;

import java.util.ArrayList;

import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public interface TranslationProcessListener {
	/**
	 * This method is called when the translation of a Job has succesfully finished.
	 * 
	 * @param productStep
	 * @param hardwareStep
	 */
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareSteps);

	/**
	 * This method is called when the translation of a Job has failed.
	 * 
	 * @param productStep
	 */
	public void onTranslationFailed(ProductStep productStep);
}
