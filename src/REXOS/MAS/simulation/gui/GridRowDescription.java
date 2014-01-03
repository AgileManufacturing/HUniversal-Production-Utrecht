package simulation.gui;

import java.util.LinkedList;

import javax.swing.DefaultListModel;

import simulation.data.EquipletDescription;

public class GridRowDescription {
	private LinkedList<EquipletDescription> equiplets = new LinkedList<EquipletDescription>();
	
	public GridRowDescription() {	}
	
	public GridRowDescription(DefaultListModel<EquipletDescription> eqs) {
		for(int i = 0; i < eqs.size(); i++) {
			equiplets.add(eqs.elementAt(i));
		}
	}
	
	public LinkedList<EquipletDescription> getEquiplets() {
		return equiplets;
	}
	
	public String toString() {
		String s = "Row: { ";
		int size = equiplets.size();
		if(size != 0) {
			for(int i = 0; i < size; i++) {
				s += ((EquipletDescription)equiplets.get(i)).getName();
				if(i < size - 1) {
					s += ", ";
				}
			}
		} else {
			s += "No Equiplets added yet";
		}
		s += " }";
		return s;
	}
}
