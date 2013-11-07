insert into ModuleType(
	manufacturer, 
	typeNumber,
	moduleTypeProperties,
	rosSoftwareBuildNumber,
	rosSoftware,
	masSoftwareBuildNumber,
	masSoftware
) values 
("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "", 1, "", 1, ""),
("The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "", 1, "", 1, ""),
("HU", "delta_robot_type_A", "", 1, "", 1, ""),
("HU", "work_plane_type_A", "{
	\"topLeftValue\" : \"_WP_TL\",
	\"topRightValue\" : \"_WP_TR\",
	\"bottomRightValue\" : \"_WP_BR\",
	\"workPlandeWidth\" : 80.0,
	\"workPlaneHeight\" : 80.0
}", 1, "", 1, "");

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
("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035", 3, 2, null, null, null, ""),
("The_Imaging_Source_Europe_GmbH", "Cheap_ass_lens", "1", 6, 4, "The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "26210035", ""),
("HU", "delta_robot_type_A", "1", 3, 2, null, null, null, ""),
("HU", "work_plane_type_A", "1", 3, 2, null, null, null, "");


