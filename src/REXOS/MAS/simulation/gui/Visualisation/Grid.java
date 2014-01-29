package simulation.gui.Visualisation;

public class Grid {

	public int currentSize = 0;
	public Equiplet[][] equiplets;

	public Grid(){
		equiplets = new Equiplet[3][3];
		for(int j = 0; j < equiplets.length; j++){
			for(int i = 0; i < equiplets[j].length; i++){
				equiplets[j][i] = new Equiplet();
				currentSize++;
			}
		}
	}

	public Grid(Equiplet[][] grid){
		equiplets = grid;
		for(int j = 0; j < equiplets.length; j++){
			for(int i = 0; i < equiplets[j].length; i++){
				equiplets[j][i] = new Equiplet();
				currentSize++;
			}
		}
	}
	
	public Grid(boolean lol){
		equiplets = new Equiplet[15][10];
		for(int j = 0; j < equiplets.length; j++){
			for(int i = 0; i < equiplets[j].length; i++){
				equiplets[j][i] = new Equiplet();
				currentSize++;
			}
		}
		equiplets[2][2] = null;
		equiplets[13][6] = null;
		equiplets[4][9] = null;
		equiplets[11][9] = null;
		equiplets[6][2] = null;
		equiplets[11][1] = null;
		equiplets[3][8] = null;
		equiplets[4][7] = null;
	}
	
	
}
