mysql -u rexos -p equiplet < src/REXOS/databases/equipletKnowledgeDatabaseDef.sql ; 
rm -r generatedOutput/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
ant generate-all ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassStewartGough ;
