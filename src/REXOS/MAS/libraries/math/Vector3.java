package libraries.math;

public class Vector3 {
    public final static Vector3 X_AXIS=new Vector3(1,0,0);
    public final static Vector3 Y_AXIS=new Vector3(0,1,0);
    public final static Vector3 Z_AXIS=new Vector3(0,0,1);
    public final static Vector3 ORIGIN=new Vector3(0,0,0);

    public final double x,y,z;
    
    public Vector3(double x, double y, double z) {
        this.x=x;
        this.y=y;
        this.z=z;
    }
    
    public double dotProduct(Vector3 v) {
        return x*v.x+y*v.y+z*v.z;
    }
    
    public Vector3 crossProduct(Vector3 v) {
        double rx=y*v.z-z*v.y;
        double ry=z*v.x-x*v.z;
        double rz=x*v.y-y*v.x;
        return new Vector3(rx, ry, rz);
    }
    
    public Vector3 newLength(double newLength) {
        double length=getLength();
        if(length==newLength) return this;
        if(length==0) return X_AXIS.newLength(newLength);
        return new Vector3(x*newLength/length, y*newLength/length, z*newLength/length);
    }
    
    public Vector3 rotationAxis() {
        return rotationAxis(X_AXIS);
    }
    
    //The rotation axis to rotate v onto this
    public Vector3 rotationAxis(Vector3 v) {
        return normalizedCrossProduct(v);
    }
    
    public Vector3 normalizedCrossProduct(Vector3 v) {
        Vector3 r=crossProduct(v);
        if(r.getLength()<0.0001) {
            r=crossProduct(X_AXIS);
        }
        if(r.getLength()<0.0001) {
            r=crossProduct(Y_AXIS);
        }
        if(r.getLength()<0.0001) {
            return X_AXIS;
        }
        return r.normalize();
    }
    
    public double angle() {
        return angleTo(X_AXIS);
    }
    
    public double angleTo(Vector3 v) {
        double cosTheta=dotProduct(v)/(getLength()*v.getLength());
        
        return Math.acos(cosTheta);
    }
    
    public double getLength() {
        return Math.sqrt(x*x+y*y+z*z);
    }        
    
    
    public Vector3 rotate(Matrix m) {
        return m.mul(this);
    }
    
    public Vector3 normalize() {
        double length=getLength();
        
        if(length==1) return this;
        if(length==0) return X_AXIS;
        return new Vector3(x/length, y/length, z/length);
    }
    
    public Vector3 addX(double a) {
        return new Vector3(x+a, y, z);
    }
    
    public Vector3 addY(double a) {
        return new Vector3(x, y+a, z);
    }
    
    public Vector3 addZ(double a) {
        return new Vector3(x, y, z+a);
    }
    
    public Vector3 add(Vector3 v) {
        return new Vector3(x+v.x, y+v.y, z+v.z);
    }
            
    public Vector3 sub(Vector3 v) {
        return new Vector3(x-v.x, y-v.y, z-v.z);
    }
    
    public Vector3 mul(double m) {
        return new Vector3(x*m, y*m, z*m);
    }
    
    public Vector3 div(double d) {
        return new Vector3(x/d, y/d, z/d);
    }
    
    public Vector3 neg() {
        return new Vector3(-x, -y, -z);
    }
    
    @Override
    public boolean equals(Object o) {
        if(!(o instanceof Vector3)) return false;
        Vector3 v=(Vector3)o;
        return v==this || sub(v).getLength() < 0.0001;
    }
    
    public String toString() {
    return String.format(java.util.Locale.ENGLISH,
        "(%.3f, %.3f, %.3f)", x, y, z);
    }
    
}