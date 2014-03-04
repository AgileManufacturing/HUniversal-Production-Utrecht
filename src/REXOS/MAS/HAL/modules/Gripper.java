package HAL.modules;

import java.util.ArrayList;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;


import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.Module;
import HAL.ModuleIdentifier;

public class Gripper extends Module {


	public Gripper(ModuleIdentifier moduleIdentifier) throws KnowledgeException {
		super(moduleIdentifier);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws KnowledgeException, KeyNotFoundException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		HardwareStep hardwareStep = new HardwareStep(compositeStep, null, moduleIdentifier);
		// TODO Auto-generated method stub
		
		hardwareSteps.addAll(getParentModule().translateCompositeStep(compositeStep));
		return hardwareSteps;
	}
}
