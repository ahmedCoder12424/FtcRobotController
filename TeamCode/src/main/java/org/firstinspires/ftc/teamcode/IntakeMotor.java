package org.firstinspires.ftc.teamcode;

public class IntakeMotor {
    public Motor intakeMotor;

    public IntakeMotor(Component intakeMotor){
        this.intakeMotor = (Motor) intakeMotor;
    }

    public void set(float speed){
        intakeMotor.setSpeed(speed);
    }

    public void down(float speed){
        intakeMotor.setSpeed(speed * -1);
    }

}


