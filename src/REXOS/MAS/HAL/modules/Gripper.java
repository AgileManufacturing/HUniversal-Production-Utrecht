package HAL.modules;

import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;

public class Gripper extends ModuleActor {
	//MONGO HOST 145.89.191.131

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
		
		JsonObject command = compositeStep.getCommand();

		
		HardwareStep hardwareStep = new HardwareStep(compositeStep, null, moduleIdentifier);
		// TODO Auto-generated method stub
		
		hardwareSteps.addAll(((ModuleActor) getParentModule()).translateCompositeStep(compositeStep));
		return hardwareSteps;
	}
}
