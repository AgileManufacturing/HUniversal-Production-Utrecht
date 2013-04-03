package dataClasses;

import java.util.ArrayList;
import java.util.List;

public class ProductionStep {
	public ArrayList<Parameter> parameters;
	public int stepId;
	
	public ProductionStep(int stepid, ArrayList<Parameter> list){
		stepId = stepid;
		parameters = list;
	}
}
