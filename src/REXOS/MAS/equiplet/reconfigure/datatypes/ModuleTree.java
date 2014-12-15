/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	ModuleTree
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	Retriefing all data from the modules on the equiplet. Returning this data as a String[][].
 *          dM@      dMMM3  .ga...g,    	@date Created:	2014-04-03
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

package MAS.equiplet.reconfigure.datatypes;

import java.util.ArrayList;

import HAL.Module;
import HAL.ModuleIdentifier;

public class ModuleTree {
	 /**
	  * @var moduleTreeList
	  * This arrayList is being used to temporarily store module information untill it is added to the ModuleTreeArray.
	  */
	private static ArrayList<String> moduleTreeList = new ArrayList<String>();
	
	 /**
	  * @var moduleTreeArray
	  * The moduleTreeArray has all the information of the modules on the equiplet.
	  * The moduleTreeArray is being returned by the getModuleTree function.
	  */
	private static String[][] moduleTreeArray;	
	
	//This will most likely be replaced later, by using the Logger-class
	private static boolean DEBUG = true;
	
	public ModuleTree(ArrayList<Module> bottomModuleList) {
		updateModuleTree(bottomModuleList);
	}
	
	 /**
	  * getModuleInformation(module)
	  * Getting the information of a module and stores it in the temp moduleTreeList.
	  * If the module has a parent, then it calls its own function again with the parent as module variable.
	  * @param a module object, containing functions to get the module information.
	  */
	private void getModuleInformation(Module module) {
		ModuleIdentifier moduleId = module.getModuleIdentifier();

		moduleTreeList.add(moduleId.getManufacturer());
		moduleTreeList.add(moduleId.getTypeNumber());
		moduleTreeList.add(moduleId.getSerialNumber());
		
		int[] modulePositions = module.getMountPosition();
		
		if(modulePositions != null) {
			moduleTreeList.add("" + modulePositions[0]);
			moduleTreeList.add("" + modulePositions[1]);
		} else {			
			try {
				if(module.getParentModule() != null) {
					getModuleInformation(module.getParentModule());
				}
			} catch (Exception e) {
				e.printStackTrace();
				if(DEBUG)	System.out.println("Gotta catch 'em all ...");
			}
		}
	}
	
	 /**
	  * updateModuleTree(bottomModuleList)
	  * Creating a moduleTree by calling the getModuleInformation function with the bottomModules.
	  * @param bottomModuleList A list that contains all the bottomModule objects.
	  */
	public void updateModuleTree(ArrayList<Module> bottomModuleList) {
				
		if(bottomModuleList != null && bottomModuleList.size() > 0) {
			if(DEBUG){
				System.out.println("Making ModuleTree");
				System.out.println("bottomModuleList Size= "+bottomModuleList.size());
			}
			moduleTreeArray = new String[bottomModuleList.size()][];			
			for(int i = 0; i < bottomModuleList.size(); i++) {			
				moduleTreeList.clear();
				Module tempModule = bottomModuleList.get(i);
				getModuleInformation(tempModule);				
				moduleTreeArray[i] = moduleTreeList.toArray(new String[moduleTreeList.size()]);
			}	
		} else {
			if(DEBUG)	System.err.println("No bottomModules, can not make tree.");	
		}
	}	
	
	 /**
	  * getModuleTree()
	  * @return the found moduleTree.
	  */
	public String[][] getModuleTree() {
		return moduleTreeArray;
	}
}
