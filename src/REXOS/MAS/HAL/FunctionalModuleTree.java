package HAL;

public class FunctionalModuleTree {
	private Mutation[] requiredMutations;
	public Mutation[] getRequiredMutations() {
		return requiredMutations;
	}
	
	public FunctionalModuleTree(Mutation[] requiredMutations) {
		this.requiredMutations = requiredMutations;
	}

}
