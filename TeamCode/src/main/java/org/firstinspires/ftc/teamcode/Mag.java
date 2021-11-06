package org.firstinspires.ftc.teamcode;

class Mag {
    long clickTime = 0l;

    private StepperServo leftMag;
    private StepperServo rightMag;
    private StepperServo midMag;

    final float initialAngleLeft = 118;
    final float raisedAngleLeft = 30;
    final float initialAngleRight = 3;
    final float raisedAngleRight = 84;
    final float flat = 15;
    final float tilted = 1;

    final float delayUp = 100;
    final float delayDown = 300;

    boolean firstClick = true;

    public Mag(Component leftMag, Component midMag, Component rightMag){
        this.leftMag = (StepperServo) leftMag;
        this.rightMag = (StepperServo) rightMag;
        this.midMag = (StepperServo) midMag;
        this.leftMag.setAngle(initialAngleLeft);
        this.rightMag.setAngle(initialAngleRight);
        this.midMag.setAngle(flat);
    }

    public void raiseControl(boolean clicked){
        if (clicked) {
            leftMag.setAngle(raisedAngleLeft);
            rightMag.setAngle(raisedAngleRight);
            if (firstClick) {
                clickTime = System.currentTimeMillis();
                firstClick = false;
            }
        }

        if (System.currentTimeMillis() - clickTime >= delayUp){
            midMag.setAngle(tilted);
            firstClick = true;
        }
    }

    public void lowerControl(boolean clicked){
        if (clicked) {
            leftMag.setAngle(initialAngleLeft);
            rightMag.setAngle(initialAngleRight);
            if (firstClick) {
                clickTime = System.currentTimeMillis();
                firstClick = false;
            }
        }

        if (System.currentTimeMillis() - clickTime >= delayDown){
            midMag.setAngle(flat);
            firstClick = true;
        }
    }
}
