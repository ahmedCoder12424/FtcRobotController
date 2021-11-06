package org.firstinspires.ftc.teamcode;

class MotionProfile{

    double targetDistance, maxV, maxA, offset;
    double t0, t1, t2, t3;
    double timeAtStart;

    public MotionProfile(double targetDistance, double maxV, double maxA, double offset, double timeAtStart){
        this.targetDistance = targetDistance - offset;
        this.maxV = maxV;
        this.maxA = maxA;
        this.offset = offset;

        this.t1 = maxV / maxA;
        this.t2 = (targetDistance / maxV) - (maxV / maxA);
        this.t3 = maxV / maxA;

        this.t0 = Math.sqrt(targetDistance / maxA);

        this.timeAtStart = timeAtStart;
    }

    public double getP(double t){

        double tFix = t;

        if (t2 < 0){
            return findAreaUntilTimeTriangle(tFix) + offset;
        } else {
            return findAreaUntilTimeTrapezoid(tFix) + offset;
        }
    }

    public double getV(double t){

        double tFix = t;

        if (t2 < 0){
            return equationSetTriangle(tFix);
        } else {
            return equationSetTrapezoid(tFix);
        }
    }

    public double getA(double t){
        return 0.0f;
    }

    private double findAreaUntilTimeTrapezoid(double t){
        if (t <= t1 && t >= 0){
            return (0.5 * t * equationSetTrapezoid(t));
        } else if (t > t1 && t <= (t1 + t2)){
            return ((0.5 * t1 * maxV) + ((t - t1) * (maxV)));
        } else if (t > (t1+t2) && t < (t1+t2+t3)){
            return ((t1 * maxV) + (t2 * maxV) + (0.5 * (t-(t1+t2+t3)) * equationSetTrapezoid(t)));
        } else if (t < 0){
            return 0;
        } else {
            return targetDistance;
        }
    }

    private double equationSetTrapezoid(double t){
        if (t <= t1 && t >= 0){
            return v1(t);
        } else if (t > t1 && t <= (t1 + t2)){
            return v2(t);
        } else if (t > (t1+t2) && t <= (t1+t2+t3)){
            return v3(t);
        } else {
            return 0;
        }
    }

    private double findAreaUntilTimeTriangle(double t){
        if (t <= t0 && t >= 0){
            return (0.5 * t * equationSetTriangle(t));
        } else if (t < (2 * t0) && t > t0){
            return ((t0*equationSetTriangle(t0)) + (0.5 * (t-(2 * t0)) * equationSetTriangle(t)));
        } else if (t < 0){
            return 0;
        } else {
            return targetDistance;
        }
    }

    private double equationSetTriangle(double t){
        if (t <= t0 && t >= 0){
            return maxA * t;
        }else if (t > t0 && t <= t0*2){
            return (-maxA * t) + (2 * maxA * t0);
        }else{
            return 0;
        }
    }


    private double v1(double t){ return maxA * t; }

    private double v2(double t){ return maxV; }

    private double v3(double t){ return (-maxA * t) + (maxA * (t1 + t2 + t3)); }
}