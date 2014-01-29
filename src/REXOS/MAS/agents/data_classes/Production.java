/**
 * @file rexos/mas/data/Production.java
 * @brief Class where all the data relevant to the production object will be
 *        saved.
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

package agents.data_classes;

import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.annotations.Expose;
import com.google.gson.annotations.SerializedName;

public class Production{
	

	private ArrayList<ProductionStep> _productionSteps;

	private ProductionEquipletMapper _prodEQMap;

	public Production(){
		_prodEQMap = new ProductionEquipletMapper();
	}

	public Production(ArrayList<ProductionStep> productionSteps)
			throws NullPointerException{
		this();
		if (productionSteps == null)
			throw new NullPointerException("Production steps can't be null");
		this._productionSteps = productionSteps;
		for(ProductionStep p : this._productionSteps){
			this._prodEQMap.addProductionStep(p.getId());
		}
	}

	public ArrayList<ProductionStep> getProductionSteps(){
		return _productionSteps;
	}
	
	public void setProductionSteps(ArrayList<ProductionStep> prodsteps) {
		this._productionSteps = prodsteps;
	}

	public void setProductionEquipletMapping(ProductionEquipletMapper prodEQMap)
			throws NullPointerException{
		if (prodEQMap == null){
			throw new NullPointerException("ProductionEquipletMapper can't be null");
		}
		this._prodEQMap = prodEQMap;
	}
	
	public ProductionStep getProductionStep(int id) {
		return _productionSteps.get(id);
	}

	public ProductionEquipletMapper getProductionEquipletMapping(){
		return this._prodEQMap;
	}
	
	@Override
	public String toString() {
	   return "DataObject [productionSteps=" + _productionSteps+ "]";
	}
	
	public int getProductionCount() {
		return this._productionSteps.size();
	}
	
	public HashMap<String, ProductionStep> createConversationIdToProductionStepMapping() {
		HashMap<String, ProductionStep> outputMap = new HashMap<String, ProductionStep>();
		for(ProductionStep step : _productionSteps) {
				outputMap.put(step.getConversationId(), step);
		}
		return outputMap;
	}
}
