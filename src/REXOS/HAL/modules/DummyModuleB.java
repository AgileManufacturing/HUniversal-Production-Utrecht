package HAL.modules;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import HAL.ModuleActor;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.OriginPlacement;
import HAL.steps.OriginPlacementType;

public class DummyModuleB extends ModuleActor {
	public DummyModuleB(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(
			CompositeStep compositeStep) throws ModuleTranslatingException,
			FactoryException, JSONException {
		// just remove everything from the compositeStep
		if(compositeStep.getCommand().has("move")) compositeStep.getCommand().remove("move");
		if(compositeStep.getCommand().has("pick")) compositeStep.getCommand().remove("pick");
		if(compositeStep.getCommand().has("place")) compositeStep.getCommand().remove("place");
		
		ArrayList<HardwareStep> steps = new ArrayList<HardwareStep>();
		steps.add(new HardwareStep(moduleIdentifier, compositeStep, new JSONObject(), 
				new OriginPlacement(OriginPlacementType.RELATIVE_TO_EQUIPLET_ORIGIN, new JSONObject())));
		return steps;
	}
}