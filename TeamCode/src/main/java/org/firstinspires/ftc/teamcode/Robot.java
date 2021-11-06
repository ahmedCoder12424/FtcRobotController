package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Component[] components;
    public Mecanum drivetrain;
    public Gyro gyro;
    public FlyWheel flywheel;
    public Intake intake;
    public WobbleGoal wobbleClaw;
    public Mag mag;
    public Flicker flicker;

    public boolean previousMotorToggle = false;
    public boolean previousServoToggle = false;
    public boolean previousWGClaw = false;
    public boolean previousWGArm = false;

    /*
    Odometer 1 = R
    Odometer 2 = L
    Odometer 3 = M
     */

    /*public Pose2d robotPose = new Pose2d(
            15.3543307,
            0.1673228,
            0.6968503935
    );*/


    public Pose robotPose = new Pose(
            Math.PI/2, Math.PI/2, 0.0,
            7.41830709, -7.41830709, 0.5748031,
            0.5748031, 0.5748031, 3.75,
            0.6968503935
            );

    //Autonomous Constants
    public float currentR = 0.0f;
    public float targetR = 0.0f;

    public float currentY = 0.0f;
    public float targetY = 0.0f;
    public float highestY = 0.0f;

    public float currentX = 0.0f;
    public float targetX = 0.0f;
    public float highestX = 0.0f;

    public float correctionX = 0.0f;
    public float correctionY = 0.0f;
    public float correctionR = 0.0f;

    public boolean pid = true;

    private final float rKPR = 0.012f;
    private final float rKIR = 0.000005f;
    private final float rKDR = 0.0000f;

    private final float yBASE = 0.5f;
    private final float xBASE = 0.06f;

    private final float yKPR = 0.06f;
    private final float yKIR = 0.000002f;
    private final float yKDR = 0.00f;

    private final float xKPR = 0.17f;
    private final float xKIR = 0.000007f;
    private final float xKDR = 0.00f;

    private float lastX = 0;
    private float lastY = 0;

    public float xCor = 0;
    public float yCor = 0;
    public float rCor = 0;

    public boolean auton = false;

    public double microstepWobble = 0;

    public boolean intakeRunning = false;


    private PIDController pidYDistance = new PIDController(0f, yKPR, yKIR, yKDR, false);
    private PIDController pidXDistance = new PIDController(0f, xKPR, xKIR, xKDR, false);
    private PIDController pidRotation = new PIDController(0.0f, rKPR, rKIR, rKDR, true);

    public MotionProfilingController mpController = new MotionProfilingController(robotPose, 0.03, 0.00005, auton);

    public boolean previousPrimeShooter = false;
    public boolean shooterPrimed = false;
    private boolean wobbleGoalClawOpen = false;
    private boolean wobbleGoalArmOpen = false;
    private boolean intakeOn = false;
    private boolean shouldLower = true;
    private boolean intakeOnForward = false;
    private boolean intakeOnReverse = false;

    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;
        mpController.auton = this.auton;
        this.components = new Component[]{
                new Motor(3, "backLeft", map, true),                            //0 left odometer
                new Motor(2, "backRight", map, false),                          //1 right odometer
                new Motor(1, "frontLeft", map, true),                           //2 middle odometer
                new Motor(0, "frontRight", map, false),                         //3
                new Motor(2, "forwardShooter", map, true),
                new Motor(3, "rearShooter", map, false),
                new StepperServo(0, "leftMag", map),
                new StepperServo(0, "midMag", map),
                new StepperServo(0, "rightMag", map),
                new StepperServo(1, "flicker", map),
                new Motor(0, "intakeFront", map, false),
                new Motor(0, "intakeRear", map, false),
                new StepperServo(0, "intakeServoFront", map),
                new StepperServo(0, "intakeServoRear", map),
                new StepperServo(0, "wobbleArm",map),
                new StepperServo(0,"wobbleArm2", map),
                new StepperServo(0, "wobbleClaw",map)
        };

        if (auton){
            drivetrain = new Mecanum(
                    components[0],
                    components[1],
                    components[2],
                    components[3],
                    true
            );
        } else {
            drivetrain = new Mecanum(
                    components[0],
                    components[1],
                    components[2],
                    components[3],
                    false
            );
        }



        this.gyro = new Gyro(map);

        this.flywheel = new FlyWheel(components[4], components[5]);

        this.mag = new Mag(components[6], components[7], components[8]);

        this.flicker = new Flicker(components[9]);

        this.intake = new Intake((StepperServo) components[12], (StepperServo) components[13], (Motor) components[10], (Motor) components[11]);

        //this.intake = new Intake(servo1, servo2, motor1, motor2);

        this.wobbleClaw = new WobbleGoal((StepperServo)components[16], (StepperServo)components[14], (StepperServo)components[15]);

        drivetrain.resetAllEncoders();

        currentR = gyro.getHeading();
        targetR = currentR;

        //lift.liftMotor2.resetEncoder();
        //fakeMotor.resetEncoder();

        currentY = getOdoY();
        currentX = getOdoX();

        pidXDistance = new PIDController(0, xKPR, xKIR, xKDR, false);
        pidYDistance = new PIDController(0, yKPR, yKIR, yKDR, false);
        pidRotation = new PIDController(0, rKPR, rKIR, rKDR, true);
    }

    public void updateLoop(){

        robotPose.updateOdometry(new double[][]{
                {(double) ((Motor) components[1]).getEncoderValue()}, //odo 1 = R
                {-(double) ((Motor) components[0]).getEncoderValue()}, //odo 2 = L
                {(double) ((Motor) components[2]).getEncoderValue()}  //odo 3 = M
        });

        currentR = gyro.getHeading();
        lastY = currentY;
        currentY = getOdoY();
        lastX = currentX;
        currentX = getOdoX();

        flywheel.updateRPM();


        double[] values = mpController.updateLoop();
        xCor = (float) values[0];
        yCor = (float) values[1];
        rCor = (float) values[2];
        if (auton) {
            drivetrain.move(xCor, yCor, rCor);
        }
    }

    public void resetMotorSpeeds(){
        drivetrain.resetMotorSpeeds();
    }

    public void stop() {
        drivetrain.stop();
    }

    public void turbo(boolean turbo){
        drivetrain.setTurbo(turbo);
    }

    public void drive(float xMove, float yMove, float rotate) {
        drivetrain.move(xMove, yMove, rotate);
    }


    public float getOdoX(){
        return 0.0f;
        //return (fakeMotor.getEncoderValue() / (8192f)) * 6.1842375f;
    }

    public float getOdoY(){
        return 0.0f;
        //return (lift.liftMotor2.getEncoderValue() / (8192f)) * 6.1842375f;
    }

    /* -- Subsystem Control -- */
    public void toggleIntake(boolean a) {
        //intake Control
        intake.deploy();

        if (a && !previousMotorToggle){
            if (intakeOn){
                intake.stop();
                intakeOn = false;
            } else {
                intake.start();
                intakeOn = true;
            }
        }
        previousMotorToggle = a;
    }

    public void intakeOn(float val) {
        if (val >= 0.5f){
            intake.start();
            intakeOnForward = true;
        } else if (!intakeOnReverse){
            intake.stop();
            intakeOnForward = false;
        }
    }

    public void intakeReverse(float val) {
        if (val >= 0.5f){
            intake.reverse();
            intakeOnReverse = true;
        } else if (!intakeOnForward){
            intake.stop();
            intakeOnReverse = false;
        }
    }

    public void primeShooter(boolean x) {
        if (x && !previousPrimeShooter){
            if (shooterPrimed){
                shouldLower = true;
                flywheel.stop();
                shooterPrimed = false;
            } else {
                shouldLower = false;
                flywheel.moveWheels();
                shooterPrimed = true;
            }
        }

        if (shouldLower) {
            mag.lowerControl(x);
        } else {
            mag.raiseControl(x);
        }

        previousPrimeShooter = x;
    }

    public void shoot(boolean b) {
        //flick
        flicker.flickControl(b);
    }

    public void powershot(boolean bump) {
        if (bump){
            flywheel.targetRPM = 4500;
        } else {
            flywheel.targetRPM = 5300;
        }
    }

    public void wobbleGoalRaise(boolean a) {
        //Toggle raise or lower wobble goals
        if(a && !previousWGArm) {
            if (wobbleGoalArmOpen) {
                wobbleClaw.armRaise();
                wobbleGoalArmOpen = false;
            } else {
                wobbleClaw.armLower();
                wobbleGoalArmOpen = true;
            }
        }
        previousWGArm = a;
    }

    public void wobbleGoalClaw(boolean b) {
        //Toggle claw open or close
        if(b && !previousWGClaw) {
            if (wobbleGoalClawOpen) {
                wobbleClaw.clawClose();
                wobbleGoalClawOpen = false;
            } else {
                wobbleClaw.clawOpen();
                wobbleGoalClawOpen = true;
            }
        }
        previousWGClaw = b;
    }

    public void microstepServo(boolean b){
        if(b){
            microstepWobble += 0.05;
            ((StepperServo)components[6]).setAngle(90+(float) microstepWobble);
        }
    }

    public static boolean tol(float current, float target, float tolerance){
        return Math.abs(current - target) <= tolerance;
    }

    public boolean stopped(boolean x){
        if(x){
            return Math.abs(lastX - currentX) <= 0.01;
        } else {
            return Math.abs(lastY - currentY) <= 0.01;
        }
    }

}
