/**
 * @file Product.java
 * @brief Class where the log and production can be retrieved from
 *        production.java
 * @date Created: 02-04-2013
 * 
 * @author Mike Schaap
 * 
 * @section LICENSE License: newBSD
 * 
 *          Copyright � 2012, HU University of Applied Sciences Utrecht. All
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

package rexos.mas.data;

import jade.core.AID;
import rexos.libraries.log.Logger;

import com.mongodb.BasicDBObject;

public class Product{
	private Production _production;


	/**
	 * @param log the log to set
	 */
	public void setLog(ProductLog log){
		this.log = log;
	}

	private ProductLog log;

	/**
	 * @return the log
	 */
	public ProductLog getLog(){
		return log;
	}

	public Product(Production production, String aid){
		if (production == null)
			Logger.log(new Exception("Production can't be null"));
		setProduction(production);
		log = new ProductLog();
	}

	/**
	 * @return the _production
	 */
	public Production getProduction(){
		return _production;
	}
	

	/**
	 * @param _production
	 *            the _production to set
	 */
	public void setProduction(Production production){
		this._production = production;
	}

	/**
	 * @param aid 
	 * @param statusData
	 */
	public void addStatusDataToLog(AID aid, BasicDBObject statusData){
		log.add(aid, statusData);
		
	}
	
	@Override
	public String toString() {
		return "DataObject [production=" +_production+"]";
	}
}
