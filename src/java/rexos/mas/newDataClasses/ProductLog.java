/**
 * @file ProductLog.java
 * @brief Class to create the productionlog which is then able to be pushed to
 *        the remote location.
 * @date Created: 02-04-2013
 * 
 * @author Theodoor de Graaff
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright © 2012, HU University of Applied Sciences Utrecht. All
 *          rights reserved.
 * 
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions
 *          are met: - Redistributions of source code must retain the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer. - Redistributions in binary form must reproduce the
 *          above copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided with
 *          the distribution. - Neither the name of the HU University of Applied
 *          Sciences Utrecht nor the names of its contributors may be used to
 *          endorse or promote products derived from this software without
 *          specific prior written permission.
 * 
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *          "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *          LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *          FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE HU
 *          UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY DIRECT,
 *          INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *          SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *          HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *          STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *          ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *          OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **/

package rexos.mas.newDataClasses;

import rexos.mas.newDataClasses.sqldatabase.sqliteDatabase;
import java.util.List;

public class ProductLog{
	private boolean writeToRemote = false;
	private boolean writeToLocal = true;
	private sqliteDatabase local;

	// private RemoteDatabaseConnection remote;
	public void add(List<LogMessage> msgs){
		if (writeToLocal){
			local.insert(msgs);
		}
		if (writeToRemote){
			// remote.insert()
		}
	}

	/**
	 * @param writeToRemote
	 * @param writeToLocal
	 * @param local
	 */
	public ProductLog(boolean writeToRemote, boolean writeToLocal,
			sqliteDatabase local){
		super();
		this.writeToRemote = writeToRemote;
		this.writeToLocal = writeToLocal;
		this.local = local;
	}

	public void pushLocalToRemote(){
		// TODO:
		// get latest remote
		// get local since latest remote
		// write to remote
	}
}
