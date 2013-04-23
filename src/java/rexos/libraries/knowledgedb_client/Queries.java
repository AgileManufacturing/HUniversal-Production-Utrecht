/**
 * @author Ammar Abdulamir
 * @file Queries.java
 * @brief Contains all queries.
 * @date Created: 2013-04-05
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package rexos.libraries.knowledgedb_client;

/**
 * Contains all queries.
 **/
public class Queries {
    /**
     * @var String MODULES_REQUIRED_PER_SERVICE
     *
     * A constant representing an sql query for fetching all modules required by a specific service. Has a service name as the only parameter.
     **/
    public static final String MODULES_REQUIRED_PER_SERVICE = "SELECT\n" +
            "    `module_groups`.`name` AS `module`\n" +
            "    , `services`.`name` AS `service`\n" +
            "FROM\n" +
            "    `rexos_knowledge_base`.`service_module`\n" +
            "    INNER JOIN `rexos_knowledge_base`.`module_groups` \n" +
            "        ON (`service_module`.`module` = `module_groups`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`services` \n" +
            "        ON (`service_module`.`service` = `services`.`id`)\n" +
            "WHERE (`services`.`name` = (?));";

    /**
     * @var String MODULES
     *
     * A constant representing an sql query for fetching all modules.
     **/
    public static final String MODULES = "SELECT \n" +
            "  `module_groups`.`name` AS `group`,\n" +
            "  `module_types`.`name` AS `type`,\n" +
            "  `modules`.`id` AS `module`,\n" +
            "  `equiplets`.`jade_address` AS `equiplet` \n" +
            "FROM\n" +
            "  `rexos_knowledge_base`.`module_types` \n" +
            "  INNER JOIN `rexos_knowledge_base`.`module_groups` \n" +
            "    ON (\n" +
            "      `module_types`.`module_group` = `module_groups`.`id`\n" +
            "    ) \n" +
            "  INNER JOIN `rexos_knowledge_base`.`modules` \n" +
            "    ON (\n" +
            "      `modules`.`module_type` = `module_types`.`id`\n" +
            "    ) \n" +
            "  LEFT JOIN `rexos_knowledge_base`.`equiplets` \n" +
            "    ON (\n" +
            "      `modules`.`location` = `equiplets`.`id`\n" +
            "    ) ;";

    /**
     * @var String PART_PROPERTIES
     *
     * A constant representing an sql query for fetching all part properties for a specific part. Has a part type name as the only paramter.
     **/
    public static final String PART_PROPERTIES = "SELECT\n" +
            "    `part_types`.`name`\n" +
            "    , `part_type_properties`.`property`\n" +
            "    , `part_type_properties`.`value`\n" +
            "    , `properties`.`name`\n" +
            "    , `properties`.`datatype`\n" +
            "FROM\n" +
            "    `rexos_knowledge_base`.`part_type_properties`\n" +
            "    INNER JOIN `rexos_knowledge_base`.`part_types` \n" +
            "        ON (`part_type_properties`.`part_type` = `part_types`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`properties` \n" +
            "        ON (`part_type_properties`.`property` = `properties`.`id`)\n" +
            "WHERE (`part_types`.`name` = (?));";

    /**
     * @var String PARTS
     *
     * A constant representing an sql query for fetching all parts.
     **/
    public static final String PARTS = "SELECT\n" +
            "    `part_types`.`name`\n" +
            "    , `parts`.`id`\n" +
            "FROM\n" +
            "    `rexos_knowledge_base`.`parts`\n" +
            "    INNER JOIN `rexos_knowledge_base`.`part_types` \n" +
            "        ON (`parts`.`type` = `part_types`.`id`);";

    /**
     * @var String POSSIBLE_STEPS_PER_EQUIPLET
     *
     * A constant representing an sql query for fetching the possible steps per equiplet. Has an equiplet address as the only parameter.
     **/
    public static final String POSSIBLE_STEPS_PER_EQUIPLET = "SELECT\n" +
            "    `equiplets`.`jade_address`\n" +
            "    , `product_steps`.`name` AS `step`\n" +
            "FROM\n" +
            "    `rexos_knowledge_base`.`modules`\n" +
            "    INNER JOIN `rexos_knowledge_base`.`equiplets` \n" +
            "        ON (`modules`.`location` = `equiplets`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`module_types` \n" +
            "        ON (`modules`.`module_type` = `module_types`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`module_groups` \n" +
            "        ON (`module_types`.`module_group` = `module_groups`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`service_module` \n" +
            "        ON (`service_module`.`module` = `module_groups`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`services` \n" +
            "        ON (`service_module`.`service` = `services`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`product_steps_services` \n" +
            "        ON (`product_steps_services`.`service` = `services`.`id`)\n" +
            "    INNER JOIN `rexos_knowledge_base`.`product_steps` \n" +
            "        ON (`product_steps_services`.`product_step` = `product_steps`.`id`)\n" +
            "WHERE (`equiplets`.`jade_address` = (?))\n" +
            "GROUP BY `step`;";

    /**
     * @var String POSSIBLE_STEPS_PER_SERVICE
     *
     * A constant representing an sql query for fetching all possible steps for all services.
     **/
    public static final String POSSIBLE_STEPS_PER_SERVICE = "SELECT \n" +
            "  `services`.`name` AS `service`,\n" +
            "  `product_steps`.`name` AS `step` \n" +
            "FROM\n" +
            "  `rexos_knowledge_base`.`product_steps_services` \n" +
            "  INNER JOIN `rexos_knowledge_base`.`product_steps` \n" +
            "    ON (\n" +
            "      `product_steps_services`.`product_step` = `product_steps`.`id`\n" +
            "    ) \n" +
            "  INNER JOIN `rexos_knowledge_base`.`services` \n" +
            "    ON (\n" +
            "      `product_steps_services`.`service` = `services`.`id`\n" +
            "    ) ;";

    /**
     * @var String SOFTWARE_FOR_MODULE
     * 
     * SQL query for retrieving the software for the specified module from the database.
     * Expects an integer parameter corresponding to the moduleId
     **/
    public static final String SOFTWARE_FOR_MODULE = "SELECT " + 
    		"    software.id AS id, " + 
    		"    software.class_name AS class_name, " + 
    		"    software.description AS description, " + 
    		"    software.jar_location AS jar_location, " + 
    		"    software.name AS name " + 
    		" FROM " + 
    		"    software " + 
    		" WHERE " + 
    		"    id = (SELECT  " + 
    		"            software " + 
    		"        FROM " + 
    		"            modules " + 
    		"                INNER JOIN " + 
    		"            module_types ON modules.module_type = module_types.id " + 
    		"        WHERE " + 
    		"            modules.id = (?))";
    
    /**
     * @var String SOFTWARE_FOR_SERVICE
     * 
     * SQL query for retrieving the software for the specified service from the database.
     * Expects an integer parameter corresponding to the serviceId
     **/
    public static final String SOFTWARE_FOR_SERVICE = "SELECT  " + 
    		"    software.id AS id, " + 
    		"    software.class_name AS class_name, " + 
    		"    software.description AS description, " + 
    		"    software.jar_location AS jar_location, " + 
    		"    software.name AS name " + 
    		" FROM " + 
    		"    software " + 
    		"        INNER JOIN " + 
    		"    services ON services.software = software.id " + 
    		" WHERE " + 
    		"    services.id = (?)";
    
    /**
     * A private constructor preventing this class to be constructed.
     **/
    private Queries() {
    }
}
