/**
 * @file src/REXOS/MAS/agents/data_classes/ProductStepScheduleInformation.java
 * @brief Object that holds information about the product schedule
 * @date Created: 04 nov 2013
 * 
 * @author Alexander Streng
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

import jade.core.AID;
import libraries.schedule.data_classes.EquipletScheduleInformation;
import libraries.schedule.data_classes.ProductStepSchedule;

public class ProductStepScheduleInformation {
	
	private ProductionStep productionStep;	
	private AID equipletAid;
	private ProductStepSchedule productStepSchedule;
	
	
	public ProductStepScheduleInformation(ProductionStep productionStep,
			AID equipletAid, ProductStepSchedule productStepSchedule) { 
		this.productionStep = productionStep;
		this.equipletAid = equipletAid;
		this.productStepSchedule = productStepSchedule;
	}
	/**
	 * @return the productionStep
	 */
	public ProductionStep getProductionStep() {
		return productionStep;
	}
	/**
	 * @param productionStep the productionStep to set
	 */
	public void setProductionStep(ProductionStep productionStep) {
		this.productionStep = productionStep;
	}
	/**
	 * @return the equipletAid
	 */
	public AID getEquipletAid() {
		return equipletAid;
	}
	/**
	 * @param equipletAid the equipletAid to set
	 */
	public void setEquipletAid(AID equipletAid) {
		this.equipletAid = equipletAid;
	}
	/**
	 * @return the productStepSchedule
	 */
	public ProductStepSchedule getProductStepSchedule() {
		return productStepSchedule;
	}
	/**
	 * @param productStepSchedule the productStepSchedule to set
	 */
	public void setProductStepSchedule(ProductStepSchedule productStepSchedule) {
		this.productStepSchedule = productStepSchedule;
	}
	
}
