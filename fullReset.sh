# clean everything up
mysql -u rexos -p equiplet < src/REXOS/databases/knowledgeDatabaseDef.sql ; 
#rm -r generatedOutput/ ; 
rm -r /tmp/rexos_model_spawner/ ; 
rm -r /tmp/rexos_node_spawner/ ; 
# copy compiled plugins to the models
cp devel/lib/libattach_plugin.so src/REXOS/models/camera/ ; 
cp devel/lib/libattach_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libattach_plugin.so src/REXOS/models/gripper/ ; 
cp devel/lib/libattach_plugin.so src/REXOS/models/lens/ ; 
cp devel/lib/libattach_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libattach_plugin.so src/REXOS/models/workplane/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libmotor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/deltaRobot/ ; 
cp devel/lib/libsensor_manager_plugin.so src/REXOS/models/sixAxis/ ; 
cp devel/lib/libgripper_plugin.so src/REXOS/models/gripper/ ; 
# export everything
ant generate-all ;
# insert parts
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader chessboard_1 chessboard ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_1	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_2	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_3	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_4	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_5	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader GC4x4MB_6	GC4x4MB ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_1	redBall		GC4x4MB_1 -16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_2	redBall		GC4x4MB_1 -5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_3	redBall		GC4x4MB_1 5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_4	redBall		GC4x4MB_1 16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_5	redBall		GC4x4MB_1 -16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_6	redBall		GC4x4MB_1 -5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_7	redBall		GC4x4MB_1 5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_8	redBall		GC4x4MB_1 16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_9	redBall		GC4x4MB_1 -16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_10	redBall		GC4x4MB_1 -5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_11	redBall		GC4x4MB_1 5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_12	redBall		GC4x4MB_1 16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_13	redBall		GC4x4MB_1 -16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_14	redBall		GC4x4MB_1 -5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_15	redBall		GC4x4MB_1 5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader redBall_16	redBall		GC4x4MB_1 16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_1	blueBall	GC4x4MB_2 -16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_2	blueBall	GC4x4MB_2 -5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_3	blueBall	GC4x4MB_2 5.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_4	blueBall	GC4x4MB_2 16.5	16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_5	blueBall	GC4x4MB_2 -16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_6	blueBall	GC4x4MB_2 -5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_7	blueBall	GC4x4MB_2 5.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_8	blueBall	GC4x4MB_2 16.5	5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_9	blueBall	GC4x4MB_2 -16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_10	blueBall	GC4x4MB_2 -5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_11	blueBall	GC4x4MB_2 5.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_12	blueBall	GC4x4MB_2 16.5	-5.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_13	blueBall	GC4x4MB_2 -16.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_14	blueBall	GC4x4MB_2 -5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_15	blueBall	GC4x4MB_2 5.5	-16.5 	10 0 0 0 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.PartRecordLoader blueBall_16	blueBall	GC4x4MB_2 16.5	-16.5 	10 0 0 0 ;
# insert equiplets
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ2 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ3 ;
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQD ;
# insert modules
java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassPickAndPlace --noTranslate EQ2;
#java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClassStewartGough --noTranslate EQ3;

