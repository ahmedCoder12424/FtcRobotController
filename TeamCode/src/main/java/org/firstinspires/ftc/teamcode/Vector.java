package org.firstinspires.ftc.teamcode;

public class Vector {
    private float theta;
    private float magnitude;
    private float x;
    private float y;


    public Vector(float t, float m){
        setTheta(t);
        setMagnitude(m);
    }

    public Vector(float x, float y, boolean b){
        this.setTheta((float)Math.atan(y/x));
        this.setMagnitude((float)Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    public Vector(){
        this(0, 0);
    }

    public float getTheta() {
        return theta;
    }

    public void setTheta(float theta) {
        this.theta = theta;
    }

    public float getMagnitude() {
        return magnitude;
    }

    public void setMagnitude(float magnitude) {
        this.magnitude = magnitude;
    }

    public void setVector(float x, float y){
        this.setMagnitude((float)Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        this.theta = (float)Math.atan(y/x);
    }

    public float getX(){
        double radians = Math.toRadians(theta);
        return magnitude * (float)Math.cos(radians);
    }

    public float getY(){
        double radians = Math.toRadians(theta);
        return magnitude * (float)Math.sin(radians);
    }

    public static Vector add(Vector v1, Vector v2){
        float Vrx = v2.getX() + v1.getX();
        float Vry = v2.getY() + v1.getY();

        Vector Vr = new Vector();
        Vr.setTheta((float)Math.atan(Vry/Vrx));
        Vr.setMagnitude((float)Math.sqrt(Math.pow(Vrx, 2) + Math.pow(Vry, 2)));

        return Vr;
    }
}
