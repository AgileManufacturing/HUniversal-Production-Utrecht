package HAL;

import HAL.Ball.BallColor;

public class Main {

	public static void main(String[] args) {

		Crate c1 = new Crate(100, 100, 4, 4);
		Crate c2 = new Crate(100, 100, 4, 4, 0, 25);

		c1.fillCrate();
		c2.fillCrate();

		System.out.println("c1:\t" + c1);
		System.out.println("c1:\tCrateLocation: " + c1.getLocation());
		System.out.println("c1:\tCrateApproachPoint: " + c1.getApproachPointForCrate());
		System.out.println("c1:\tCrateRelativeApproachPoint: " + c1.getRelativeApproachPointForCrate());
		
		Ball b1 = c1.getBallFromIndex(0, 0);
		System.out.println("\nb1:\t" + b1);
		System.out.println("b1:\tBallLocationInCrate: " + c1.getBallLocation(0, 0));
		System.out.println("b1:\tBallApproachPoint: " + c1.getApproachPointFromBall(0, 0));
		System.out.println("b1:\tBallRelativeApproachPoint: " + c1.getRelativeApproachPointForBall(0, 0));
		

		System.out.println("--------------------------------------------------------------------------");

		System.out.println("c2:\t" + c2);
		System.out.println("c2:\tCrateLocation:" + c2.getLocation());
		System.out.println("c2:\tCrateApproachPoint:" + c2.getApproachPointForCrate());
		System.out.println("c2:\tCrateRelativeApproachPoint:" + c2.getRelativeApproachPointForCrate());
		
		Ball b2 = c2.getBallFromIndex(0, 0);
		System.out.println("\nb2:\t" + b2);
		System.out.println("b2:\tBallLocationInCrate: " + c2.getBallLocation(0, 0));
		System.out.println("b1:\tBallApproachPoint: " + c2.getApproachPointFromBall(0, 0));
		System.out.println("b1:\tBallRelativeApproachPoint: " + c2.getRelativeApproachPointForBall(0, 0));
	}
}

class Ball {

	final int BALL_DIMENSIONS = 5;

	enum BallColor{
		RED,BLUE,GREEN,YELLOW
	}

	int x, y, z;
	BallColor color;

	public Ball(int x, int y, BallColor color){
		this.x = x;
		this.y = y;
		z = BALL_DIMENSIONS;

		this.color = color;
	}

	public Vector getLocation(){
		return new Vector(x, y, z);
	}

	public String toString(){
		return "Ball:[x:" + x + ", y:" + y + "] color:" + color;
	}

}

class Crate {

	final int CRATE_HEIGHT = 15;
	final int APPROACH_OFFSET = 40;
	final int MAX_ROTATION_IN_DEGREES = 90;
	
	final int DISTANCE_BETWEEN_BALLS = 10;
	final int DISTANCE_BETWEEN_BORDER_AND_BALL = 5;
	
	int x, y, z;
	int degreesRotatedX = 0;
	int degreesRotatedY = 0;

	int amountOfBallsInWidth, amountOfBallsInDepth;

	Ball[] balls;

	public Crate(int x, int y, int amountOfBallsInWidth, int amountOfBallsInDepth){
		this.x = x;
		this.y = y;
		z = CRATE_HEIGHT;

		this.amountOfBallsInWidth = amountOfBallsInWidth;
		this.amountOfBallsInDepth = amountOfBallsInDepth;

		balls = new Ball[amountOfBallsInDepth * amountOfBallsInWidth];
	}

	public Crate(int x, int y, int amountOfBallsInWidth, int amountOfBallsInDepth, int degreesRotatedX, int degreesRotatedY){
		this.x = x;
		this.y = y;
		z = CRATE_HEIGHT;

		this.amountOfBallsInWidth = amountOfBallsInWidth;
		this.amountOfBallsInDepth = amountOfBallsInDepth;

		balls = new Ball[amountOfBallsInDepth * amountOfBallsInWidth];

		this.degreesRotatedX = degreesRotatedX;
		this.degreesRotatedY = degreesRotatedY;
	}

	public void fillCrate(){
		for(int y = 0; y < amountOfBallsInDepth; y++){
			for(int x = 0; x < amountOfBallsInWidth; x++){
				balls[y*amountOfBallsInWidth + x] = new Ball(DISTANCE_BETWEEN_BORDER_AND_BALL + x * DISTANCE_BETWEEN_BALLS, 
						DISTANCE_BETWEEN_BORDER_AND_BALL + y * DISTANCE_BETWEEN_BALLS, 
						BallColor.RED
				);
			}
		}
	}

	public void fillCrate(Ball[] balls){
		for(int i = 0; i < this.balls.length; i++){
			this.balls[i] = balls[i];
		}
	}

	public Ball[] getBalls(){
		return balls;
	}

	public Ball getBallFromIndex(int x, int y){
		return balls[y*amountOfBallsInWidth + x];
	}

	public Vector getLocation(){
		return new Vector(x, y, z);
	}

	public Vector getBallLocation(int x, int y){
		return new Vector(balls[y*amountOfBallsInWidth + x].x + this.x, 
				balls[y*amountOfBallsInWidth + x].y + this.y, 
				balls[y*amountOfBallsInWidth + x].z + z
				);
	}

	public Vector getApproachPointForCrate(){
		//TODO improve ..?
		Vector temp = getRelativeApproachPointForCrate();
		
		temp.x += x;
		temp.y += y;
		temp.z += z;

		return temp;
	}

	public Vector getApproachPointFromBall(int x, int y){
		//TODO improve ..?
		Vector temp = getRelativeApproachPointForBall(x, y);

		temp.x += this.x;
		temp.y += this.y;
		temp.z += this.z;
		
		return temp;
	}

	public Vector getRelativeApproachPointForCrate(){
		Vector relativeApproach = new Vector(0, 0, APPROACH_OFFSET);
		return relativeApproach.rotate(degreesRotatedX, degreesRotatedY);
	}

	public Vector getRelativeApproachPointForBall(int x, int y){
		Vector relativeApproachForBall = getBallFromIndex(x, y).getLocation();
		relativeApproachForBall.z += APPROACH_OFFSET;
		return relativeApproachForBall.rotate(degreesRotatedX, degreesRotatedY);
	}

	public String toString(){
		return "Crate:[x:" + x + ", y:" + y + ", z:" + z + ", amountOfBalls:" + balls.length + ", degreesX:"+degreesRotatedX + ", degreesY:" + degreesRotatedY + "]";
	}

}

class Vector {

	public int x, y, z;

	public Vector(int x, int y, int z){
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public Vector rotate(double degreesX, double degreesY){
		double a, b, c;
		double p, q, r;
		double u, v, w;
		a = b = c = p = q = r = u = v = w = 0;
		
		if(degreesX != 0 && degreesY == 0){
			a = 1; b = 0; c = 0;
			p = 0; q = Math.cos(Math.toRadians(degreesX)); r = -Math.sin(Math.toRadians(degreesX));
			u = 0; v = -Math.sin(Math.toRadians(degreesX)); w = Math.cos(Math.toRadians(degreesX));
		} else if(degreesX == 0 && degreesY != 0){
			a = Math.cos(Math.toRadians(degreesY)); b = 0; c = Math.sin(Math.toRadians(degreesY));
			p = 0; q = 1; r = 0;
			u = -Math.sin(Math.toRadians(degreesY)); v = 0; w = Math.cos(Math.toRadians(degreesY));
		} else if(degreesX != 0 && degreesY != 0){
			//TODO
			System.out.println("Unfinished!");
		} else {
			return this;
		}
		return new Vector((int)Math.round(a*x+b*y+c*z), (int)Math.round(p*x+q*y+r*z), (int)Math.round(u*x+v*y+w*z));
	}

	public String toString(){
		return "Vector[x:" + x + ", y:" + y + ", z:" + z + "]";
	}
}