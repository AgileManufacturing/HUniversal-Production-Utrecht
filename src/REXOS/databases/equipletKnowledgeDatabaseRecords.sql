insert into ModuleType(
	manufacturer, 
	typeNumber,
	moduleTypeProperties,
	rosSoftwareBuildNumber,
	rosSoftware,
	masSoftwareBuildNumber,
	masSoftware
) values 
("ManA", "TypeA", "", 1, "", 1, "");

insert into Module(
	manufacturer,
	typeNumber,
	serialNumber,
	mountPointX,
	mountPointY,
	attachedToManufacturer,
	attachedToTypeNumber,
	attachedToSerialNumber,
	moduleProperties
) values 
("ManA", "TypeA", "SerA", 3, 2, null, null, null, ""),
("ManA", "TypeA", "SerB", 6, 4, "ManA", "TypeA", "SerA", ""),
("ManA", "TypeA", "SerC", 12, 8, "ManA", "TypeA", "SerA", "");


