package org.firstinspires.ftc.teamcode;

class MotionController {
    Pose robotPose;

    double reqX = 0, reqY = 0, reqTheta = 0;
    double correctionX = 0, correctionY = 0, correctionTheta = 0;
    double currentX = 0, currentY = 0, currentTheta = 0;

    double[] correctionVals = new double[3];
    final double xKP = 1, xKI = 1, xKD = 1;
    final double yKP = 1, yKI = 1, yKD = 1;
    final double rKP = 1, rKI = 1, rKD = 1;


    private PIDController pidYDistance = new PIDController(0f, yKP, yKI, yKD, false);
    private PIDController pidXDistance = new PIDController(0f, xKP, xKI, xKD, false);
    private PIDController pidRotation = new PIDController(0.0f, rKP, rKI, rKD, true);

    private boolean inMotion = true;


    public MotionController(Pose robotPose) {
        this.robotPose = robotPose;
    }

    public void updateRequestedPose(double x, double y, double theta){
        this.reqX = x;
        this.reqY = y;
        this.reqTheta = theta;
    }

    public void setInMotion(boolean inMotion) {
        this.inMotion = inMotion;
    }

    public double[] updateLoop(){
        this.currentX = robotPose.getX();
        this.currentY = robotPose.getY();
        this.currentTheta = robotPose.getHeading();

        if(inMotion){
            correctionX = pidXDistance.update(currentX);
            correctionY = pidYDistance.update(currentY);
            correctionTheta = pidRotation.update(currentTheta);

            correctionVals[0] = correctionX;
            correctionVals[1] = correctionY;
            correctionVals[2] = correctionTheta;
        } else {
            correctionVals[0] = 0;
            correctionVals[1] = 0;
            correctionVals[2] = 0;
        }

        return correctionVals;
    }
}
