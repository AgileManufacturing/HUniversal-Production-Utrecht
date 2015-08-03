package HAL;

import java.util.ArrayList;
import java.util.concurrent.Semaphore;

import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.MastListener;
import HAL.listeners.TestProcessListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public class HardwareAbstractionLayer extends AbstractHardwareAbstractionLayer implements MastListener, TestProcessListener {
	protected ShadowHardwareAbstractionLayer shadowHal;
	protected Semaphore shadowHalSemaphore = new Semaphore(1);

	public HardwareAbstractionLayer(String equipletName, HardwareAbstractionLayerListener hardwareAbstractionLayerListener)
			throws KnowledgeException, BlackboardUpdateException {
		this(equipletName, hardwareAbstractionLayerListener, false);
	}
	public HardwareAbstractionLayer(String equipletName, HardwareAbstractionLayerListener hardwareAbstractionLayerListener, boolean useShadow)
			throws KnowledgeException, BlackboardUpdateException {
		super(equipletName, false, hardwareAbstractionLayerListener);
		if(useShadow == true) {
			shadowHal = new ShadowHardwareAbstractionLayer(equipletName, this, this);
		} else {
			shadowHal = null;
		}
	}
	
	@Override
	public void onTranslationFinished(ProductStep step, ArrayList<HardwareStep> hardwareSteps) {
		synchronized (translationProcesses) {
			translationProcesses.remove(step);
		}
		if(shadowHal != null) {
			try {
				shadowHalSemaphore.acquire();
				shadowHal.testHardwareSteps(hardwareSteps, step);
			} catch (InterruptedException ex) {
				Logger.log(LogSection.HAL, LogLevel.ERROR, "Acquiring semaphore lock failed");
			}
		} else {
			translationProcessListener.onTranslationFinished(step, hardwareSteps);
		}
	}
	@Override
	public void onTestFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareSteps) {
		shadowHalSemaphore.release();
		translationProcessListener.onTranslationFinished(productStep, hardwareSteps);
	}

	@Override
	public void onTestFailed(ProductStep productStep) {
		shadowHalSemaphore.release();
		translationProcessListener.onTranslationFailed(productStep);
	}
	@Override
	public void shutdown() {
		super.shutdown();
		if(shadowHal != null) {
			shadowHal.shutdown();
		}
	}
	
	@Override
	public boolean insertModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		return super.insertModule(jsonStaticSettings, jsonDynamicSettings);
	}
	@Override
	public boolean updateModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		return super.updateModule(jsonStaticSettings, jsonDynamicSettings);
	}
	@Override
	public JSONObject deleteModule(ModuleIdentifier moduleIdentifier) throws Exception {
		return super.deleteModule(moduleIdentifier);
	}
	@Override
	public void translateProductStep(String service, JSONObject criteria) {
		super.translateProductStep(service, criteria);
	}
	@Override
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps) {
		super.executeHardwareSteps(hardwareSteps);
	}
}
