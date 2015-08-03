mysql -u rexos -psoxer -e "DROP DATABASE equiplet"
mysql -u rexos -psoxer -e "CREATE DATABASE equiplet"
mysql -u rexos -psoxer equiplet < equipletKnowledgeDatabaseDef.sql
mysql -u rexos -psoxer equiplet < equipletRecord.sql
