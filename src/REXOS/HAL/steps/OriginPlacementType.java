package HAL.steps;

public enum OriginPlacementType {
    RELATIVE_TO_IDENTIFIER("relativeToIdentifier"),
    RELATIVE_TO_CURRENT_POSITION("relativeToCurrentPosition"),
    RELATIVE_TO_MODULE_ORIGIN("relativeToModuleOrigin"),
    RELATIVE_TO_EQUIPLET_ORIGIN("relativeToEquipletOrigin");
    
    private String name;
    
    private OriginPlacementType(String name) {
    	this.name = name;
    }
    public String getName() {
    	return name;
    }
}
