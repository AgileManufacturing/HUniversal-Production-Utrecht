package Visualisation;

import java.awt.Color;
import java.awt.Graphics;

public class DrawGrid {

	private static final int WIDTH = 30;
	private static final int HEIGHT = 30;
	private static final int SPACING_X = 10;
	private static final int SPACING_Y = 10;
	
	private int x;
	private int y;
	private char id;

	private Grid grid;

	public DrawGrid(){
		x = 0;
		y = 0;
		id = '0';

		grid = new Grid();
	}

	public DrawGrid(DrawGrid e){
		x = e.x;
		y = e.y;
		id = e.id;

		grid = new Grid();
	}

	public DrawGrid(int x, int y){
		this.x = x;
		this.y = y;
		id = '@';

		grid = new Grid(true);
	}

	public boolean draw(Graphics g){

		for(int j = 0; j < grid.equiplets.length; j++){
			for(int i = 0; i < grid.equiplets[j].length; i++){
				if(grid.equiplets[j][i] != null){
					g.setColor(Color.BLUE);
					x = i * (WIDTH + SPACING_X) + SPACING_X;
					g.fillRect(x-(WIDTH/2), y-(HEIGHT/2), WIDTH, HEIGHT);
					g.setColor(Color.BLACK);
					g.drawChars(new char[] {'E', 'Q', id}, 0, 3, x-(WIDTH/2), y);
				}
			}
			y = j * (HEIGHT + SPACING_Y) + SPACING_Y;
		}


		return true;
	}

}
