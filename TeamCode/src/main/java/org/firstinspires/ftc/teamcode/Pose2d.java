package org.firstinspires.ftc.teamcode;

public class Pose2d {

    /*
    R = radius of odometry wheel (in)
     */
    private double R;

    private double trackwidth;
    private double forwardOffset;

    /*
        x,y = coordinates of the robot (in)
        heading = current direction of the robot (degrees)
            0 is forward
            90 is right
            -90 is left
         */
    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;
    private double time = System.currentTimeMillis();

    public double getXVelocity() {
        return xVelocity;
    }

    public void setxVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setyVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public double getXAcceleration() {
        return xAcceleration;
    }

    public void setxAcceleration(double xAcceleration) {
        this.xAcceleration = xAcceleration;
    }

    public double getYAcceleration() {
        return yAcceleration;
    }

    public void setyAcceleration(double yAcceleration) {
        this.yAcceleration = yAcceleration;
    }

    private double xVelocity;
    private double yVelocity;
    private double deltaXVelocity;
    private double deltaYVelocity;
    private double xAcceleration;
    private double yAcceleration;


    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return (heading * 180) / Math.PI;
    }

    public double[][] deltaThetas;

    public double[][] encoderTicks = new double[][]{
            {0},
            {0},
            {0}
    };
    public double[][] deltaEncoderTicks = new double[][]{
            {0},
            {0},
            {0}
    };
    public double[][] oldTicks = new double[][]{
            {0},
            {0},
            {0}
    };

    /*
    trackwidth = distance between right and left odometry pods; = 15.3543307
    forwardOffset = distance from center of rotation to the middle odometry pods; = 0.1673228
     */
    public Pose2d(double trackwidth, double forwardOffset, double R){
        this.R = R;

        this.trackwidth = trackwidth;
        this.forwardOffset = forwardOffset;
    }

    /*
    Changes an input value (encoder ticks) to a radian output
     */
    private double encoderToRad(double encVal){
        return (encVal/8192) * 2 * Math.PI;
    }

    /*
    Changes a matrix of encoder ticks to a matrix of radians
     */
    private double[][] changeToRadians(double[][] matrixTemp){
        double[][] matrixToChange = matrixTemp;

        for (int i = 0; i < matrixToChange.length; i++){
            for (int j = 0; j < matrixToChange[i].length; j++){
                matrixToChange[i][j] = encoderToRad(matrixToChange[i][j]);
            }
        }
        return matrixToChange;
    }

    /*
    dTheta1, dTheta2, dTheta3 = change in rotation of each odometry pod (encoder ticks)

    Matrix deltaThetas:
    | dTheta1 |
    | dTheta2 |
    | dTheta3 |
     */
    public void updateOdometry(double[][] encoderTicks) {

        this.encoderTicks = encoderTicks;

        this.deltaEncoderTicks[0][0] = this.encoderTicks[0][0] - this.oldTicks[0][0]; //R 0 1
        this.deltaEncoderTicks[1][0] = this.encoderTicks[1][0] - this.oldTicks[1][0]; //L 1 2
        this.deltaEncoderTicks[2][0] = this.encoderTicks[2][0] - this.oldTicks[2][0]; //M 2 3

        deltaThetas = changeToRadians(this.deltaEncoderTicks);

        double delta_left_encoder_pos = this.deltaEncoderTicks[1][0];
        double delta_right_encoder_pos = this.deltaEncoderTicks[0][0];
        double delta_center_encoder_pos = this.deltaEncoderTicks[2][0];

        double phi = (R * (delta_left_encoder_pos - delta_right_encoder_pos)) / trackwidth;
        double delta_middle_pos = (R * (delta_left_encoder_pos + delta_right_encoder_pos)) / 2;
        double delta_perp_pos = R * (delta_center_encoder_pos - forwardOffset * phi);

        double deltaY = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
        double deltaX = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);

        this.deltaXVelocity = (deltaX/(System.currentTimeMillis()-this.time)) - this.xVelocity;
        this.deltaYVelocity = (deltaY/(System.currentTimeMillis()-this.time)) - this.yVelocity;
        this.xVelocity = (deltaX/(System.currentTimeMillis()-this.time));
        this.yVelocity = (deltaY/(System.currentTimeMillis()-this.time));

        this.xAcceleration = (deltaXVelocity/(System.currentTimeMillis()-this.time));
        this.yAcceleration = (deltaYVelocity/(System.currentTimeMillis()-this.time));

        x += deltaX;
        y += deltaY;
        heading += phi;

        this.time = System.currentTimeMillis();
        this.oldTicks = encoderTicks;
    }
}
