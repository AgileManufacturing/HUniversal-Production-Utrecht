mysql -u rexos -p equiplet < src/REXOS/databases/equipletKnowledgeDatabaseDef.sql ; 
#rm -r generatedOutput/ ; 
rm -r /tmp/rexos_model_spawner/ ; 
rm -r /tmp/rexos_node_spawner/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libgripper_plugin.so src/REXOS/models/gripper/ ; 
ant generate-all ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ2;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ3;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQD;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassPickAndPlace ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassStewartGough ;

