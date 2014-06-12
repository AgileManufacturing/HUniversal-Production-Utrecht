package HAL.exceptions;

import HAL.steps.CompositeStep;

public class ModuleTranslatingException extends Exception {
	private static final long serialVersionUID = 3386162951426886138L;
	
	private CompositeStep compositeStep;

	public ModuleTranslatingException(String message, CompositeStep compositeStep) {
		super(message);
		this.compositeStep = compositeStep;
	}
	public ModuleTranslatingException(String message, Throwable throwable, CompositeStep compositeStep) {
		super(message, throwable);
		this.compositeStep = compositeStep;
	}

	public CompositeStep getCompositeStep() {
		return compositeStep;
	}

}
