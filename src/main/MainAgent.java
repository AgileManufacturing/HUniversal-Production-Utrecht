/**
 * @file MainAgent.java
 * @brief Dummy main agent for testing purposes. Starts up the different agents
 *        and sets the (for now) hardcoded information.
 * @date Created: 02-04-2013
 * 
 * @author Alexander Streng
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

package main;

import jade.core.AID;
import jade.core.Agent;

import java.util.ArrayList;

import newDataClasses.Parameter;
import newDataClasses.ParameterGroup;
import newDataClasses.ParameterList;
import newDataClasses.Product;
import newDataClasses.Production;
import newDataClasses.ProductionStep;
import newDataClasses.ProductionStepStatus;

@SuppressWarnings("serial")
public class MainAgent extends Agent{
	@Override
	protected void setup(){
		try{
			System.out.println("Starting agents:");
			// instantiate the 'equiplet' agents
			Object[] EquipletArray = new Object[]{1};// pickndplace
			getContainerController().createNewAgent("EQ1",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{2}; // colour
			getContainerController().createNewAgent("EQ2",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{3}; // rotate
			getContainerController().createNewAgent("EQ3",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{2}; // rotate
			getContainerController().createNewAgent("EQ4",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{1}; // rotate
			getContainerController().createNewAgent("EQ5",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{3}; // rotate
			getContainerController().createNewAgent("EQ6",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{1}; // rotate
			getContainerController().createNewAgent("EQ7",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{1}; // rotate
			getContainerController().createNewAgent("EQ8",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{2}; // rotate
			getContainerController().createNewAgent("EQ9",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = new Object[]{3}; // rotate
			getContainerController().createNewAgent("EQ10",
					"testingAgents.EquipletAgent", EquipletArray).start();
			EquipletArray = null;
			// Create a parameterlist
			ParameterList parameterList = new ParameterList();
			ParameterGroup p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "1"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "2"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);
			// Create productionsteps
			ProductionStep stp1 = new ProductionStep(1, 0, parameterList);
			stp1.setStatus(ProductionStepStatus.STATE_TODO);
			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "3"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "4"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);
			ProductionStep stp2 = new ProductionStep(2, 1, parameterList);
			stp2.setStatus(ProductionStepStatus.STATE_TODO);
			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "5"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "6"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);
			ProductionStep stp3 = new ProductionStep(3, 2, parameterList);
			stp3.setStatus(ProductionStepStatus.STATE_TODO);
			p = new ParameterGroup("Color"); // group colour
			p.add(new Parameter("Id", "7"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("Shape"); // group shape
			p.add(new Parameter("Id", "8"));
			parameterList.AddParameterGroup(p);
			p = new ParameterGroup("loc"); // group location
			p.add(new Parameter("x", "2"));
			p.add(new Parameter("y", "2"));
			parameterList.AddParameterGroup(p);
			ProductionStep stp4 = new ProductionStep(4, 3, parameterList);
			stp4.setStatus(ProductionStepStatus.STATE_TODO);
			// The product agent will receive several steps to handle.
			ArrayList<ProductionStep> stepList = new ArrayList<>();
			stepList.add(stp1);
			stepList.add(stp2);
			stepList.add(stp3);
			stepList.add(stp4);
			Production production = new Production(stepList);
			Product product = new Product(production, new AID("pa1",
					AID.ISLOCALNAME).toString());
			// We need to pass an Object[] to the createNewAgent.
			// But we only want to pass our product!
			Object[] arg = new Object[1];
			arg[0] = product;
			getContainerController().createNewAgent("pa1",
					"productAgent.ProductAgent", arg).start();
		} catch(Exception e){
			e.printStackTrace();
			doDelete();
		}
	}
}
