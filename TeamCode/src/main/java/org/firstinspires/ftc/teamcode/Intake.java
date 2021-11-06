package org.firstinspires.ftc.teamcode;

public class Intake {
    public StepperServo intakeServo1;
    public StepperServo intakeServo2;
    public Motor intakeMotor1;
    public Motor intakeMotor2;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, Motor intakeMotor1,
                  Motor intakeMotor2){
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
    }

    public void deploy(){
        intakeServo1.setAngle(0);
        intakeServo2.setAngle(0);
    }

    public void start(){
        intakeMotor1.setSpeed(0.4f);
        intakeMotor2.setSpeed(0.4f);
    }

    public void reverse(){
        intakeMotor1.setSpeed(-0.4f);
        intakeMotor2.setSpeed(-0.4f);
    }

    public void stop(){
        intakeMotor1.setSpeed(0);
        intakeMotor2.setSpeed(0);
    }

}
