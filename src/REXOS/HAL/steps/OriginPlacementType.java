package HAL.steps;

public enum OriginPlacementType {
    RELATIVE_TO_CURRENT_POSITION("RELATIVE_TO_CURRENT_POSITION"),
    RELATIVE_TO_MODULE_ORIGIN("RELATIVE_TO_MODULE_ORIGIN"),
    RELATIVE_TO_EQUIPLET_ORIGIN("RELATIVE_TO_EQUIPLET_ORIGIN"),
    RELATIVE_TO_PART_ORIGIN("RELATIVE_TO_PART_ORIGIN"),
    RELATIVE_TO_WORLD_ORIGIN("RELATIVE_TO_WORLD_ORIGIN"),
    UNDEFINED("UNDEFINED");
    
    private String name;
    
    private OriginPlacementType(String name) {
    	this.name = name;
    }
    public String getName() {
    	return name;
    }
}
