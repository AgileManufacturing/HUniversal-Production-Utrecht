package util.log;

public enum LogSection {
	NONE(null, ""),
	HAL(null, "hal"),
		HAL_BLACKBOARD(HAL, "halBb"),
		HAL_FACTORIES(HAL, "halFact"),
			HAL_MODULE_FACTORY(HAL_FACTORIES, "halModFact"),
				HAL_MODULE_FACTORY_SQL(HAL_MODULE_FACTORY, "halModFactSql"),
			HAL_CAPABILITY_FACTORY(HAL_FACTORIES, "halCapFact"),
				HAL_CAPABILITY_FACTORY_SQL(HAL_CAPABILITY_FACTORY, "halCapFactSql"),
		HAL_TRANSLATION(HAL, "halTransl"),
		HAL_EXECUTION(HAL, "halexec"),
		HAL_MODULES(HAL, "halMod"),
		HAL_CAPABILITIES(HAL, "halCap"),
		HAL_RECONFIG(HAL, "halReconf"),
			HAL_RECONFIG_SQL(HAL_RECONFIG, "halReconfSql"),
	MAS(null, "mas"),
	MAS_EQUIPLET_AGENT(MAS, "masEqA"),
	MAS_PRODUCT_AGENT(MAS, "masPA"),
	MAS_PRODUCT_AGENT_SCHEDULING(MAS_PRODUCT_AGENT, "masPASched"),
	MAS_SUPPLY_AGENT(MAS, "masSA");
	
	
	private LogSection parentSection;
	private String name;
	
	private LogSection(LogSection parentSection, String name) {
		this.parentSection = parentSection;
		this.name = name;
	}
	
	public LogSection getParentSection() {
		return parentSection;
	}
	public String getName() {
		return name;
	}
}
