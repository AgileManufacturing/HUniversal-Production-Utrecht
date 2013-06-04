/**
 * @file rexos/libraries/blackboard_client/MongoOperation.java
 * @brief Enum representing the different CRUD operations in MongoDB.
 * @date Created: 2012-04-04
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

package rexos.libraries.blackboard_client;

/**
 * Enum listing the different CRUD operations in MongoDB, as well as their internal representation in the oplog.
 *
 **/
public enum MongoOperation {
	/**
	 * Insert operation.
	 **/
	INSERT("i"),
	/**
	 * Update operation.
	 **/
	UPDATE("u"),
	/**
	 * NOOP operation.
	 **/
	NOOP("n"),
	/**
	 * Delete operation.
	 **/
	DELETE("d");
	
	/**
	 * @var String opCode
	 * Internal representation in the "op" field of the oplog for this operation.
	 **/
	private final String opCode;

	/**
	 * Constructs the MongoOperation with the specified opcode.
	 * @param opCode Internal representation in the "op" field of the oplog for this operation.
	 **/
	private MongoOperation(String opCode) {
		this.opCode = opCode;
	}

	/**
	 * Returns the opcode for this operation.
	 * @return The opcode for this operation.
	 **/
	public String getOpCode() {
		return opCode;
	}
	
	/**
	 * Attempts to find the {@link MongoOperation} corresponding to the given opCode.
	 * @param opCode The opcode for which to find the MongoOperation
	 * @return The MongoOperation corresponding to the given opCode or null.
	 **/
	public static MongoOperation get(String opCode) {
		for (MongoOperation op : values()) {
			if (op.opCode.equals(opCode)) {
				return op;
			}
		}
		
		return null;
	}
}
