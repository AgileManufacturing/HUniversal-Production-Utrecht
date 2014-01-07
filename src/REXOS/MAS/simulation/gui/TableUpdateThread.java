/**                                     ______  _______   __ _____  _____
 *                  ...++,              | ___ \|  ___\ \ / /|  _  |/  ___|
 *                .+MM9WMMN.M,          | |_/ /| |__  \ V / | | | |\ `--.
 *              .&MMMm..dM# dMMr        |    / |  __| /   \ | | | | `--. \
 *            MMMMMMMMMMMM%.MMMN        | |\ \ | |___/ /^\ \\ \_/ //\__/ /
 *           .MMMMMMM#=`.gNMMMMM.       \_| \_|\____/\/   \/ \___/ \____/
 *             7HMM9`   .MMMMMM#`		
 *                     ...MMMMMF .      
 *         dN.       .jMN, TMMM`.MM     	@file 	TableUpdateThread.java
 *         .MN.      MMMMM;  ?^ ,THM		@brief 	...
 *          dM@      dMMM3  .ga...g,    	@date Created:	2013-12-19
 *       ..MMM#      ,MMr  .MMMMMMMMr   
 *     .dMMMM@`       TMMp   ?TMMMMMN   	@author	Alexander Hustinx
 *   .dMMMMMF           7Y=d9  dMMMMMr    
 *  .MMMMMMF        JMMm.?T!   JMMMMM#		@section LICENSE
 *  MMMMMMM!       .MMMML .MMMMMMMMMM#  	License:	newBSD
 *  MMMMMM@        dMMMMM, ?MMMMMMMMMF    
 *  MMMMMMN,      .MMMMMMF .MMMMMMMM#`    	Copyright � 2013, HU University of Applied Sciences Utrecht. 
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
package simulation.gui;

import java.awt.MouseInfo;
import java.awt.Point;
import java.util.List;

import javax.swing.JTable;
import javax.swing.RowSorter;
import javax.swing.SwingWorker;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;

import simulation.Simulation;
import simulation.mas_entities.Equiplet;
import simulation.mas_entities.Grid;

class TableUpdateThread extends SwingWorker<Void, Void> {

	public static final int PLACEHOLDER = 0;

	private EquipletListFrame elf;
	private JTable jTable;
	
	private boolean isPaused = false;

	public TableUpdateThread(EquipletListFrame elf) {
		super();
		this.elf = elf;
		System.out.println("[DEBUG]\t\tCreated TableUpdateThread");
		
		jTable = elf.getJTable();
	}

	@Override
	public Void doInBackground() {
		while (!isCancelled()) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			if (!isPaused) {
				updateTable();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		return null;
	}

	// @ TODO Add actual data to the Table ...
    public void updateTable(){
    	jTable = elf.getJTable();
    	Grid grid = elf.getGrid();
    	System.out.println("grid " + grid);
    	Equiplet[] equiplets = grid.getEquiplets();
    	
    	DefaultTableModel model = (DefaultTableModel) jTable.getModel();
    	
    	System.out.println(equiplets);
    	for(int i = 0; i < equiplets.length; i++) {
    		if(i >= model.getRowCount()) {
    			model.addRow(new Object[]{"", "", 0.0, 0});
    		}
    		
    		model.setValueAt(equiplets[i].getName(), i, 0);
    		model.setValueAt(equiplets[i].getEquipletState().toString(), i, 1);
    		model.setValueAt((equiplets[i].getLoad() * 100), i, 2);
    		model.setValueAt(equiplets[i].getBatchReservation(), i, 3);
    	}
    }
	
	public void pause() {
		isPaused = true;
		System.out.println("[DEBUG]\t\tPaused TableUpdateThread");
	}

	public void resume() {
		isPaused = false;
		System.out.println("[DEBUG]\t\tResumed TableUpdateThread");
	}
	
	public boolean isPaused(){
		return isPaused;
	}

	protected void done() {
		System.out.println("[DEBUG]\t\tCancelled TableUpdateThread");
	}
}
