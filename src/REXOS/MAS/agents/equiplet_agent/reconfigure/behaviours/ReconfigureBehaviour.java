/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ReconfigureBehaviour
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	Responsible for setting up and shutting down the server to allow reconfiguration of the equiplet.
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-05-12
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Tom Oosterwijk
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright © 2014, HU University of Applied Sciences Utrecht. 
 *  JMMMMMMMm.    MMMMMM#!.MMMMMMMMM'.		All rights reserved.
 *   WMMMMMMMMNNN,.TMMM@ .MMMMMMMM#`.M  
 *    JMMMMMMMMMMMN,?MD  TYYYYYYY= dM     
 *                                        
 *	Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *	- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 *   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
package agents.equiplet_agent.reconfigure.behaviours;

import java.net.Inet4Address;
import java.net.UnknownHostException;

import javax.jws.WebParam;
import javax.jws.WebService;
import javax.xml.ws.Endpoint;

import agents.equiplet_agent.EquipletAgent;
import agents.equiplet_agent.reconfigure.ModuleDataManager;
import agents.equiplet_agent.reconfigure.datatypes.ModuleTree;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import HAL.HardwareAbstractionLayer;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;

import jade.core.behaviours.Behaviour;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KnowledgeException;

public class ReconfigureBehaviour extends Behaviour{
	public HardwareAbstractionLayer HAL;
	public EquipletAgent equipletAgent;
	public boolean serverHosted = false;
	public ReconfigureBehaviour(HardwareAbstractionLayer hal, EquipletAgent ea) {
		super();
		this.HAL = hal;
		equipletAgent = ea;
	}

	@Override
	public void action() {
		QrReceiver qr = new QrReceiver();

		// TODO Auto-generated method stub
		if(serverHosted){
			System.out.println("ServerHosted");
			if(!((EquipletAgent.machineState.equals("OFFLINE") || EquipletAgent.machineState.equals("SAVE") ) 
					&& EquipletAgent.machineMode.equals("SERVICE"))){				
				try {
					Endpoint.publish("http://" + Inet4Address.getLocalHost().getHostAddress() + 
							":9191/QrReceiver", qr).stop();
					equipletAgent.register();
				} catch (UnknownHostException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				serverHosted=false;
			}
	        
		}else{
			if(((EquipletAgent.machineState.equals("OFFLINE") || EquipletAgent.machineState.equals("SAVE") ) 
					&& EquipletAgent.machineMode.equals("SERVICE"))){
				 try {
						Endpoint.publish("http://" + Inet4Address.getLocalHost().getHostAddress() + 
									":9191/QrReceiver", qr);
						System.out.println("http://" + Inet4Address.getLocalHost().getHostAddress() + 
								":9191/QrReceiver published");
						equipletAgent.deregister();
					} catch (UnknownHostException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (KnowledgeException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				 serverHosted=true;
			}
		}
	}
	@Override
	public boolean done() {
		// TODO Auto-generated method stub
		return false;
	}
	
	@WebService
	public class QrReceiver {
		
		 /**
		  * @var moduleTree
		  * The object is being used to create a moduleTree based on the bottomModules from the HAL.
		  */
		private ModuleTree moduleTree;
		
		
		 /**
		  * getModules()
		  * Recieving all the bottomModules from HAL.
		  * Send the bottomModules to the moduleTree to create a Tree.
		  * @return String[][] array that contains the information about all the modules that the equiplet contains.
		 * @throws JarFileLoaderException 
		 * @throws FactoryException 
		  */
		public String[][] getModules() throws FactoryException, JarFileLoaderException {
			moduleTree = new ModuleTree(HAL.getBottomModules());
			
			return moduleTree.getModuleTree();
		}

		 /**
		  * addModule(String)
		  * Adding a new module to the equiplet.
		  * @param The module name as a string type.
		  * The module name is being used to retrieve the right data.
		  * This is done by the ModuleDataManager.
		  * The returning string data from the ModuleDataManager is casted to a JSON object and then send to the HAL.
		  */
		public void addModule(@WebParam(name="moduleDataJson") String moduleDataJson) {
			try {
				if(moduleDataJson != null) {
					JsonObject moduleSettings = new JsonParser().parse(moduleDataJson).getAsJsonObject();
					ModuleDataManager moduleDataManager = new ModuleDataManager(moduleSettings.get("qrCode").getAsString());				
					JsonObject staticSettings = new JsonParser().parse(moduleDataManager.getJsonFileAsString()).getAsJsonObject();
					JsonObject dynamicSettings = new JsonObject();
					dynamicSettings.add("attachedTo", moduleSettings.get("attachedTo"));
					dynamicSettings.add("mountPointX", moduleSettings.get("mountPointX"));
					dynamicSettings.add("mountPointY", moduleSettings.get("mountPointY"));
					System.out.println("Static: "+staticSettings.toString());
					System.out.println("Static: "+dynamicSettings.toString());
					HAL.insertModule(staticSettings, dynamicSettings);
				}
			} catch (Exception e) {
				e.printStackTrace();
			}

		}
		 /**
		  * editModule(String)
		  * Editing a existing module.
		  * @param The module name as a string type.
		  * The module name is being used to let the HAL know what module needs to be updated.
		  */
		public void editModule(@WebParam(name="moduleDataJson") String moduleDataJson) {
			/*
			 * @TODO
			 * Change the Module information
			 * Send the information to the HAL
			 */
		}
		
		
		 /**
		  * removeModule(String)
		  * removeModule a  module from the equiplet.
		  * @param The module name as a string type.
		  * The module name is being used to retrieve the right from the HAL.
		  * The returning data from the HAL is written away to the USB file.
		  * This is done by the ModuleDataManager.
		  */
		public void removeModule(@WebParam(name="moduleDataJson") String moduleDataJson) {

			try {
				if(moduleDataJson != null) {
					JsonObject moduleSettings = new JsonParser().parse(moduleDataJson).getAsJsonObject();
					String[] splittedQrString = moduleSettings.get("qrCode").getAsString().split("\\|");				

					ModuleIdentifier moduleIdentifier = new ModuleIdentifier(splittedQrString[1], splittedQrString[2], splittedQrString[3]);
					JsonObject moduleData = HAL.deleteModule(moduleIdentifier);
					if(moduleData != null){
						ModuleDataManager moduleDataManager = new ModuleDataManager(moduleSettings.get("qrCode").getAsString());
						moduleDataManager.writeStringToJsonFile(moduleData.toString());			
					}
				}

			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

}
