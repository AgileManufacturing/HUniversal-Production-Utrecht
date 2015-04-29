#!/usr/bin/env sh
echo -n "Choose number of equiplets: "
read numberOfEquiplets
# clean everything up
mysql -u rexos -p equiplet < src/REXOS/databases/knowledgeDatabaseDef.sql ; 
ant HAL ;


for((i=0;i<numberOfEquiplets;i++)) do 
	java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.EquipletRecordLoader EQ$i ;
	java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClass --noTranslate EQ$i $i ;
	rosrun equiplet_node equiplet_node --scadaPort $((8000 + $i)) EQ$i &
	rosrun node_spawner_node node_spawner_node EQ$i &
	rosrun environment_cache environment_cache EQ$i &
	java -cp build/REXOS/:src/REXOS/external_libraries/* HAL.testerClasses.HALTesterClass --noInsert EQ$i $i &
done
