package org.firstinspires.ftc.teamcode;

public class Pose {

    /*
    R = radius of odometry wheel (in)
     */
    private double R;

    /*
    Matrix C:
    | a1, b1, c1 |
    | a2, b2, c2 |
    | a3, b3, c3 |
     */
    public double[][] C;
    public double[][] CInverse;

    double[][] soln;


    /*
        x,y = coordinates of the robot (in)
        heading = current direction of the robot (degrees)
            0 is forward
            90 is right
            -90 is left
         */
    private double x = 0.0;
    private double y = 0.0;
    private double xVelocity = 0;
    private double yVelocity = 0;
    private double deltaXVelocity = 0;
    private double deltaYVelocity = 0;
    private double xAcceleration = 0;
    private double yAcceleration = 0;
    private double heading = 0.0;
    private double time;


    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getXVelocity() {
        return xVelocity;
    }


    public double getYVelocity() {
        return yVelocity;
    }

    public double getXAcceleration() {
        return xAcceleration;
    }

    public double getYAcceleration() {
        return yAcceleration;
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
    t1, t2, t3 = orientation of each odometry pod (radians)

    x1, x2, x3 = X-coordinates of each odometry pod from center (in)

    y1, y2, y3 = Y-coordinates of each odometry pod from center (in)
     */
    public Pose(double t1, double t2, double t3, double x1, double x2, double x3, double y1, double y2, double y3, double R){
        this.time = System.currentTimeMillis();
        C = new double[][]{
                {Math.cos(t1), Math.sin(t1), x1 * Math.sin(t1) -  y1 * Math.cos(t1)},
                {Math.cos(t2), Math.sin(t2), x2 * Math.sin(t2) -  y2 * Math.cos(t2)},
                {Math.cos(t3), Math.sin(t3), x3 * Math.sin(t3) -  y3 * Math.cos(t3)}
        };
        CInverse = Matrix.inverse(C);

        this.R = R;
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

        this.deltaEncoderTicks[0][0] = this.encoderTicks[0][0] - this.oldTicks[0][0];
        this.deltaEncoderTicks[1][0] = this.encoderTicks[1][0] - this.oldTicks[1][0];
        this.deltaEncoderTicks[2][0] = this.encoderTicks[2][0] - this.oldTicks[2][0];


        deltaThetas = changeToRadians(this.deltaEncoderTicks);
        this.soln = Matrix.multiply(CInverse, deltaThetas);

        double deltaPerp = soln[0][0] * R;
        double deltaMiddle = soln[1][0] * R;
        double deltaHeading = soln[2][0] * R;

        double deltaY = deltaMiddle * Math.cos(heading) - deltaPerp * Math.sin(heading);
        double deltaX = deltaMiddle * Math.sin(heading) + deltaPerp * Math.cos(heading);

        this.deltaXVelocity = (deltaX/(System.currentTimeMillis()-this.time)) - this.xVelocity;
        this.deltaYVelocity = (deltaY/(System.currentTimeMillis()-this.time)) - this.yVelocity;
        this.xVelocity = (deltaX/(System.currentTimeMillis()-this.time));
        this.yVelocity = (deltaY/(System.currentTimeMillis()-this.time));

        this.xAcceleration = (deltaXVelocity/(System.currentTimeMillis()-this.time));
        this.yAcceleration = (deltaYVelocity/(System.currentTimeMillis()-this.time));

        x = x + deltaX;
        y = y + deltaY;
        heading = heading + deltaHeading;

        this.time = System.currentTimeMillis();

        this.oldTicks = encoderTicks;
    }
}
