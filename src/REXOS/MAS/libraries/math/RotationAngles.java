package libraries.math;
/**
 * Struct describing the rotation angles in the 3D space
 * @author Tommas Bakker
 *
 */
public class RotationAngles {
	public double pitch, roll, yaw;
	
	public RotationAngles() {
		pitch = 0;
		roll = 0;
		yaw = 0;
	}
	public RotationAngles(double pitch, double roll, double yaw) {
		this.pitch = pitch;
		this.roll = roll;
		this.yaw = yaw;
	}
	
	public Matrix generateRotationMatrix() {
		Matrix rotationMatrix = Matrix.IDENTITY;
		rotationMatrix.rotX(pitch);
		rotationMatrix.rotY(roll);
		rotationMatrix.rotZ(yaw);
		
		return rotationMatrix;
	}
	
	@Override
	public String toString() {
		return "{pitch: " + pitch + "roll: " + roll + "yaw: " + yaw + "}";
	}
}
