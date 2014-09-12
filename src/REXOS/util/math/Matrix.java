package util.math;

// source: http://pastebin.com/Gq4mHL82

public class Matrix {
    public final static Matrix IDENTITY= new Matrix(Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS);
    public final Vector3 xAxis, yAxis, zAxis;
    
    public Matrix(double ... e) {
        if(e.length!=9) throw new RuntimeException();
        xAxis = new Vector3(e[0], e[3], e[6]);
        yAxis = new Vector3(e[1], e[4], e[7]);
        zAxis = new Vector3(e[2], e[5], e[8]);
    }
    
    public Matrix(Vector3 xAxis, Vector3 yAxis, Vector3 zAxis) {
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.zAxis = zAxis;
    }
    
    public Matrix(Vector3 axis, double theta) {
        Vector3 u=axis.normalize();
        double sin=Math.sin(theta);
        double cos=Math.cos(theta);
        double uxy=u.x*u.y*(1-cos);
        double uyz=u.y*u.z*(1-cos);
        double uxz=u.x*u.z*(1-cos);
        double ux2=u.x*u.x*(1-cos);
        double uy2=u.y*u.y*(1-cos);
        double uz2=u.z*u.z*(1-cos);
        double uxsin=u.x*sin;
        double uysin=u.y*sin;
        double uzsin=u.z*sin;
        
        xAxis = new Vector3(cos+ux2, uxy+uzsin, uxz-uysin);
        yAxis = new Vector3(uxy-uzsin, cos+uy2, uyz+uxsin);
        zAxis = new Vector3(uxz+uysin, uyz-uxsin, cos+uz2);
    }
    
    static public Matrix xRotationMatrix(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Matrix(Vector3.X_AXIS,
                          new Vector3(0, cos, sin),
                          new Vector3(0, -sin, cos));
    }
    
    static public Matrix yRotationMatrix(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Matrix(new Vector3(cos, 0, -sin),
                          Vector3.Y_AXIS,
                          new Vector3(sin, 0, cos));
    }
    
    static public Matrix zRotationMatrix(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Matrix(new Vector3(cos, sin, 0),
                          new Vector3(-sin, cos, 0),
                          Vector3.Z_AXIS);
    }
    
    public Matrix rotX(double theta) {
        return xRotationMatrix(theta).mul(this);
    }
    
    public Matrix rotY(double theta) {
        return yRotationMatrix(theta).mul(this);
    }
    
    public Matrix rotZ(double theta) {
        return zRotationMatrix(theta).mul(this);
    }
    
    public Matrix mul(double d) {
        return new Matrix(xAxis.mul(d), yAxis.mul(d), zAxis.mul(d));
    }
    
    public Vector3 mul(Vector3 v) {
        return xAxis.mul(v.x).add(yAxis.mul(v.y)).add(zAxis.mul(v.z));
    }
    
    private Matrix mul(Matrix m) {
        return new Matrix(mul(m.xAxis), mul(m.yAxis), mul(m.zAxis));
    }
    
    public Matrix rotateRel(Matrix m) {
        return mul(m);
    }
    
    public Matrix rotateAbs(Matrix m) {
        return m.mul(this);
    }
    
    public Matrix normalize() {
        Vector3 vz=xAxis.crossProduct(yAxis);
        Vector3 vy=vz.crossProduct(xAxis);        
        return new Matrix(xAxis.normalize(),
                          vy.normalize(),
                          vz.normalize());
    }
    
    public Matrix transpose() {
        return new Matrix(xAxis.x, xAxis.y, xAxis.z,
                          yAxis.x, yAxis.y, yAxis.z,
                          zAxis.x, zAxis.y, zAxis.z);
    }
    
    public double determinant() {
        return xAxis.x*(yAxis.y*zAxis.z-zAxis.y*yAxis.z) -
               yAxis.x*(zAxis.z*xAxis.y-zAxis.y*xAxis.z) +
               zAxis.x*(xAxis.y*yAxis.z-yAxis.y*xAxis.z);
    }
    
    public Matrix inverse() {
        Vector3 A = yAxis.crossProduct(zAxis);
        Vector3 B = zAxis.crossProduct(xAxis);
        Vector3 C = xAxis.crossProduct(yAxis);
        return new Matrix(A,B,C).transpose().mul(1/determinant());
    }
    
    public Matrix oppositeRotMatrix() {
        return transpose();
    }
    
    @Override
    public boolean equals(Object o) {
        if(!(o instanceof Matrix)) return false;
        Matrix m=(Matrix)o;
        return m==this || xAxis.equals(m.xAxis) && yAxis.equals(m.yAxis) && zAxis.equals(m.zAxis);
    }
    
    public String toString() {
        return String.format(java.util.Locale.ENGLISH,
            "[%.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f]",
            xAxis.x, yAxis.x, zAxis.x,
            xAxis.y, yAxis.y, zAxis.y,
            xAxis.z, yAxis.z, zAxis.z);
    }
    
}