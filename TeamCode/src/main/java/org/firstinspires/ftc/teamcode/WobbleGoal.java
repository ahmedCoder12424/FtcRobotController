package org.firstinspires.ftc.teamcode;

public class WobbleGoal {
    public StepperServo wgClaw;
    public StepperServo wgArm;
    public StepperServo wgArm2;

    public WobbleGoal(StepperServo wgClaw, StepperServo wgArm, StepperServo wgArm2){
        this.wgClaw = wgClaw;
        this.wgArm = wgArm;
        this.wgArm2 = wgArm2;

        wgClaw.setAngle(35);
        wgArm.setAngle(10);
        wgArm2.setAngle(155);
    }

    public void armRaise(){
        wgArm.setAngle(0);
        wgArm2.setAngle(165);
    }

    public void armLower(){
        wgArm.setAngle(155);
        wgArm2.setAngle(10);
    }

    public void clawClose(){
        wgClaw.setAngle(0);
    }

    public void clawOpen(){
        wgClaw.setAngle(35);
    }
}
