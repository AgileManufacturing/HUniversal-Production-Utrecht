package HAL.steps;

public enum OriginPlacementType {
    RELATIVE_TO_IDENTIFIER("RELATIVE_TO_IDENTIFIER"),
    RELATIVE_TO_CURRENT_POSITION("RELATIVE_TO_CURRENT_POSITION"),
    RELATIVE_TO_MODULE_ORIGIN("RELATIVE_TO_MODULE_ORIGIN"),
    RELATIVE_TO_EQUIPLET_ORIGIN("RELATIVE_TO_EQUIPLET_ORIGIN");
    
    private String name;
    
    private OriginPlacementType(String name) {
    	this.name = name;
    }
    public String getName() {
    	return name;
    }
}
