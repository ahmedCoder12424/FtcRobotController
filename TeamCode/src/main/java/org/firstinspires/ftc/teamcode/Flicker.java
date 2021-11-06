package org.firstinspires.ftc.teamcode;

public class Flicker {

    public StepperServo flickerServo;

    long timeOfClick = 1l;

    public final float initialAngle = 56;
    final float extendedAngle = 0;

    final float delay = 600;

    boolean firstClick = true;
    public boolean moveback = false;

    public Flicker(Component servo){
        this.flickerServo = (StepperServo) servo;
        flickerServo.setAngle(initialAngle);
    }

    //Run when button pressed
    public void flickControl(boolean clicked){
        if (clicked) {
            if (firstClick && System.currentTimeMillis() - timeOfClick >= delay){
                flickerServo.setAngle(extendedAngle);
                timeOfClick = System.currentTimeMillis();
                firstClick = false;
                moveback = true;
            }
        }

        if (moveback && System.currentTimeMillis() - timeOfClick >= (delay/2)){
            flickerServo.setAngle(initialAngle);
            firstClick = true;
            moveback = false;
        }
    }
}