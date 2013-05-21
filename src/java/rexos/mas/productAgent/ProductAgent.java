/**
 * @file ProductAgent.java
 * @brief Initial product agent creation for testing purposes
 * @date Created: 08-04-2013
 * 
 * @author Alexander Streng
 * @author Arno Derks
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

package java.rexos.mas.productAgent;

import java.rexos.mas.newDataClasses.Product;
import java.rexos.mas.newDataClasses.ProductionStep;

import jade.core.AID;
import jade.core.Agent;
import libraries.blackboardJavaClient.src.nl.hu.client.BlackboardClient;
import libraries.blackboardJavaClient.src.nl.hu.client.InvalidDBNamespaceException;
import libraries.blackboardJavaClient.src.nl.hu.client.InvalidJSONException;

public class ProductAgent extends Agent{
	private static final long serialVersionUID = 1L;
	// Private fields
	private Product _product;
	private OverviewBehaviour _overviewBehaviour;
	// CID variables
	private static int _convIDCnt = 0;
	private String _convIDBase;
	public int prodStep = 0;

	@Override
	protected void setup(){
		try{
			_product = (Product) getArguments()[0];
			_overviewBehaviour = new OverviewBehaviour();
			addBehaviour(_overviewBehaviour);
			System.out.println("I spawned as a product agent");
		} catch(Exception e){
			System.out.println("Productagent exited with: " + e.getMessage());
			doDelete();
		}
	}

	/*
	 * Generates an unique conversation id based on the agents localname, the
	 * objects hashcode and the current time.
	 */
	public String generateCID(){
		if (_convIDBase == null){
			_convIDBase = getLocalName() + hashCode()
					+ System.currentTimeMillis() % 10000 + "_";
		}
		return _convIDBase + (_convIDCnt++);
	}

	public void reschedule(){
		_overviewBehaviour.reschedule();
	}

	public void rescheduleAndRemoveEquiplet(){
		removeEquiplet(getAID());
		// remove equiplet first
		_overviewBehaviour.reschedule();
	}

	public Product getProduct(){
		return this._product;
	}

	// This function is for testing purposes only and will later be replaced
	// within the Equiplet Agent its functionality
	@SuppressWarnings("static-method")
	public void removeEquiplet(AID aid){
		BlackboardClient bbc = new BlackboardClient("145.89.191.131", 27017);
		// try to remove the given 'aid' from the blackboard
		try{
			bbc.removeDocuments(aid.toString());
		} catch(InvalidJSONException | InvalidDBNamespaceException e){
			e.printStackTrace();
		}
	}

	public void setProduct(Product value){
		this._product = value;
	}

	public void outPutProductStepList(){
		for(ProductionStep stp : this.getProduct().getProduction()
				.getProductionSteps()){
			for(AID aid : this.getProduct().getProduction()
					.getProductionEquipletMapping()
					.getEquipletsForProductionStep(stp.getId()).keySet()){
				System.out
						.println("Step: " + stp.getId() + " has equiplets:\n");
				System.out.println(aid.getLocalName());
			}
		}
	}
}
