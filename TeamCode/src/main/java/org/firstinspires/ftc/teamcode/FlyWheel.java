package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class FlyWheel {

    public Motor shooter1;
    public Motor shooter2;
    
    private long lastTime;
    
    private double pastTicks;
    private double pastTicks2;

    public double targetRPM = 5300;

    public double lastTarget = 5300;

    final double KP = 0.0005, KI = 0.00, KD = 0.00;

    PIDController flywheelPID = new PIDController(targetRPM, KP, KI, KD, false);
    PIDController flywheelPID2 = new PIDController(targetRPM, KP, KI, KD, false);

    public double shooter1Speed = 0.0;
    public double shooter2Speed = 0.0;
    
    private double shooterFF = 0.7;
    public double currentRPM = 0;
    public double currentRPM2 = 0;

    boolean flywheelSpin = false;

    public FlyWheel(Component shooter1, Component shooter2){
        this.shooter1 = (Motor) shooter1;
        this.shooter2 = (Motor) shooter2;

        shooter1Speed = 1;
        shooter2Speed = 1;
        
        lastTime = System.currentTimeMillis();
        
        pastTicks = this.shooter1.getEncoderValue();
        pastTicks2 = this.shooter2.getEncoderValue();
    }
    
    public void moveWheels(){
        flywheelSpin = true;
    }

    public void updateRPM(){

        if (lastTarget != targetRPM){
            flywheelPID = new PIDController(targetRPM, KP, KI, KD, false);
            flywheelPID2 = new PIDController(targetRPM, KP, KI, KD, false);
        }

        long time = System.currentTimeMillis();
        double currentTicks = shooter1.getEncoderValue();
        double currentTicks2 = shooter2.getEncoderValue();
        lastTarget = targetRPM;

        currentRPM = ((shooter1.getEncoderValue() - pastTicks) / (time - lastTime)) * ((60000 * 40)/(28*22));
        currentRPM2 = ((shooter2.getEncoderValue() - pastTicks2) / (time - lastTime)) * ((60000 * 40)/(28*22));

        lastTime = time;

        pastTicks = currentTicks;
        pastTicks2 = currentTicks2;

        double correction1 = flywheelPID.update(currentRPM);
        double correction2 = flywheelPID2.update(currentRPM2);

        shooter1Speed = correction1 + shooterFF;
        shooter2Speed = correction2 + shooterFF;

        if (flywheelSpin){
            shooter1.setSpeed((float) Range.clip(shooter1Speed, -1, 1));
            shooter2.setSpeed((float) Range.clip(shooter1Speed, -1, 1));
        } else {
            shooter1.setSpeed(0);
            shooter2.setSpeed(0);
        }
    }

    public void resetMotorSpeeds(){
        shooter1Speed = 0.7;
        shooter2Speed = 0.7;
    }

    public void resetAllEncoders(){
        shooter1.resetEncoder();
        shooter2.resetEncoder();
    }

    public void stop(){
        flywheelSpin = false;
    }
}