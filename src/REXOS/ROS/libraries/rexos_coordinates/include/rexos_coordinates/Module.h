/**
 * @file Module.h
 * @brief Coordinate system for communication between nodes
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
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
 *
 **/

#pragma once

#include <vectors/Vectors.h>
#include <rexos_knowledge_database/Module.h>
#include "rexos_logger/rexos_logger.h"

/**
 * @brief this class provides a system to translate coordinates from module coordinates to equiplet coordinates.
 */
namespace rexos_coordinates {
	class Module {
	private:
		rexos_knowledge_database::Module* module;
		
		Vector3 moduleToEquiplet;
		Vector3 equipletToModule;
		
	public:
		/**
		 * Converts a module coordinate to an equiplet coordinate, using the moduleToEquiplet vector
		 */
		Vector3 convertToEquipletCoordinate(Vector3 moduleCoordinate);
		/**
		 * Converts a module coordinate to an equiplet coordinate, using the moduleToEquiplet vector
		 */
		Vector4 convertToEquipletCoordinate(Vector4 moduleCoordinate);
		/**
		 * Converts an equiplet coordinate to a module coordinate, using the equipletToModule vector
		 */
		Vector3 convertToModuleCoordinate(Vector3 equipletCoordinate);
		/**
		 * Converts an equiplet coordinate to a module coordinate, using the equipletToModule vector
		 */
		Vector4 convertToModuleCoordinate(Vector4 equipletCoordinate);
		
		/**
		 * Constructor, reads the module information from the database and updates the local properties. \n
		 *
		 * @param representation of the module in the knowledge database.
		 */
		Module(rexos_knowledge_database::Module* module);
	public:
		/**
		 * Reads the module information from the database and updates the local properties. \n
		 * Use this function if you have physically moved the module
		 */
		void updateTranslationVectors();
	};
}
