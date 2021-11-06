package org.firstinspires.ftc.teamcode;

public class MotionProfilePIDController {
    private double lastError;
    private double setPoint;
    private double errorSum;

    private MotionProfile motionProfile;

    private double kp, ki, kd, kv, ka;

    private long lastTime;

    private long initTime;

    /* Constructor */
    public MotionProfilePIDController(MotionProfile motionProfile, double kp, double ki, double kd, double kv, double ka){
        this.motionProfile = motionProfile;

        this.lastError = 0;
        this.lastTime = System.currentTimeMillis();
        this.errorSum = 0;

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kv = kv;
        this.ka = ka;

        this.initTime = System.currentTimeMillis();
    }

    public float update(double newInput){

        long time = System.currentTimeMillis();
        long period = time - lastTime;
        double error;

        error = motionProfile.getP(time - initTime) - newInput;

        if ((int)Math.signum(lastError) != (int) Math.signum(error)) {
            errorSum = 0;
        }

        errorSum +=  (error * period);
        double derError = (error - lastError) / period;

        double output = (kp * error) + (ki * errorSum) + (kd * derError) + (kv * motionProfile.getV(time - initTime)) + (ka * motionProfile.getA(time - initTime));

        lastError = error;
        lastTime = time;
        return (float) output;
    }
}