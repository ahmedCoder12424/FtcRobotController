package main.java.org.firstinspires.ftc.teamcode;

public class Arm {
    
    public StepperServo armServo;

    public int initialAngle = 0;            /* arbitrary values for now */
    public int desiredAngle = 45;                                                   

    public Arm(Component servo){
        this.armServo = (StepperServo) servo;
        armServo.setAngle(initialAngle);
    }

    public void onClick(boolean click){
        if(click){
            armServo.setAngle(desiredAngle);
        } else {
            armServo.setAngle(initialAngle);
        }
    }

}
