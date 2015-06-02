package HAL;

import java.util.ArrayList;

import org.json.JSONObject;

import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.EquipletCommandListener;
import HAL.listeners.ExecutionProcessListener;
import HAL.listeners.MastListener;
import HAL.listeners.TestProcessListener;
import HAL.steps.HardwareStep;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TestProcess;

public class ShadowHardwareAbstractionLayer extends AbstractHardwareAbstractionLayer {
	TestProcessListener testProcessListener;
	EquipletCommandListener forwardTarget;

	public ShadowHardwareAbstractionLayer(String equipletName, MastListener mastListener, TestProcessListener testProcessListener)
			throws KnowledgeException, BlackboardUpdateException {
		super(equipletName, true, mastListener, mastListener, null, null, null);
		this.equipletCommandListener = this;
		this.testProcessListener = testProcessListener;
	}
	
	public void testHardwareSteps(ArrayList<HardwareStep> hardwareSteps, JSONObject criteria) {
		TestProcess testProcess = new TestProcess(this, hardwareSteps, criteria);
		testProcess.start();
	}
	
	@Override
	public boolean insertModule(JSONObject jsonStaticSettings,
			JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		// nothing required to happen as the non-shadow HAL will update the KDB
		return true;
	}
	@Override
	public boolean updateModule(JSONObject jsonStaticSettings,
			JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		// nothing required to happen as the non-shadow HAL will update the KDB
		capabilityFactory.checkCache();
		moduleFactory.checkCache();
		return true;
	}
	@Override
	public JSONObject deleteModule(ModuleIdentifier moduleIdentifier)
			throws Exception {
		// nothing required to happen as the non-shadow HAL will update the KDB
		capabilityFactory.checkCache();
		moduleFactory.checkCache();
		return (JSONObject) JSONObject.NULL;
	}
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps, ExecutionProcessListener listener) {
		ExecutionProcess executionProcess = new ExecutionProcess(listener, hardwareSteps, moduleFactory);
		executionProcess.start();
	}
	public void spawnPartModel(JSONObject equipletCommand, EquipletCommandListener listener) {
		this.forwardTarget = listener;
		this.getRosInterface().postEquipletCommand(equipletCommand);
	}
	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		this.forwardTarget.onEquipletCommandStatusChanged(status);
		this.forwardTarget = null;
	}
}
