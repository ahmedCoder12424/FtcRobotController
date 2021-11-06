package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group="Juice TeleOp")
public class Main extends OpMode{

    Robot robot;
    double maxXVel = 0;
    double maxYVel = 0;
    double maxXAccel = 0;
    double maxYAccel = 0;

    double lastEnc = 0;
    double lastTime = System.currentTimeMillis();

    /*
    GAMEPAD CONTROLS:
    GAMEPAD 1:
        left stick x = drive horizontal
        left stick y = drive vertical
        right stick x = rotation
        left trigger =
        right trigger =
        left bumper =
        right bumper =
        a =
        b =
        x
        y
        dpad
     */


    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        ///telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Init", "Robot created");

        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());

        telemetry.addData("maxXVelocity", maxXVel);
        telemetry.addData("maxYVelocity", maxYVel);
        telemetry.addData("maxXAcceleration", maxXAccel);
        telemetry.addData("maxYAcceleration", maxYAccel);
        telemetry.addData("Middle Odometer", (double) ((Motor) robot.components[2]).getEncoderValue());
        telemetry.addData("Left Odometer", (double) ((Motor) robot.components[0]).getEncoderValue());
        telemetry.addData("Right Odometer", -(double) ((Motor) robot.components[1]).getEncoderValue());
    }

    public void start(){
        //robot.lift.liftMotor2.resetEncoder();
        //robot.fakeMotor.resetEncoder();
    }

    @Override
    public void loop() {
        robot.updateLoop(); //DISABLE FOR COMPETITION

        robot.turbo(gamepad1.right_bumper);

        robot.intakeOn(gamepad1.right_trigger);

        robot.intakeReverse(gamepad1.left_trigger);

        robot.primeShooter(gamepad1.x);

        robot.shoot(gamepad1.b);

        robot.wobbleGoalRaise(gamepad2.a);
        //telemetry.addData("micro", robot.microstepWobble);
        //robot.microstepServo(gamepad2.y);

        robot.wobbleGoalClaw(gamepad2.b);

        robot.powershot(gamepad1.left_bumper);


        if(gamepad1.dpad_down){
            robot.drive(0, 0.4f, 0f);
        }else if(gamepad1.dpad_up){
            robot.drive(0, -0.4f, 0f);
        } else if (gamepad1.dpad_right) {
            robot.drive(0.4f, 0f, 0f);
        } else if (gamepad1.dpad_left) {
            robot.drive(-0.4f, 0f, 0f);
        } else {
            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -1*gamepad1.right_stick_x);
        }
        if (robot.robotPose.getXVelocity() > maxXVel) {
            maxXVel = robot.robotPose.getXVelocity();
        }
        if (robot.robotPose.getYVelocity() > maxYVel) {
            maxYVel = robot.robotPose.getYVelocity();
        }
        if (robot.robotPose.getXAcceleration() > maxXAccel) {
            maxXAccel = robot.robotPose.getXAcceleration();
        }
        if (robot.robotPose.getYAcceleration() > maxYAccel) {
            maxYAccel = robot.robotPose.getYAcceleration();
        }

        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());

        telemetry.addData("maxXVelocity", maxXVel);
        telemetry.addData("maxYVelocity", maxYVel);
        telemetry.addData("maxXAcceleration", maxXAccel);
        telemetry.addData("maxYAcceleration", maxYAccel);

        telemetry.addData("Middle Odometer", (double) ((Motor) robot.components[2]).getEncoderValue());
        telemetry.addData("Left Odometer", (double) ((Motor) robot.components[0]).getEncoderValue());
        telemetry.addData("Right Odometer", -(double) ((Motor) robot.components[1]).getEncoderValue());

        telemetry.addData("Flywheel 1 RPM", robot.flywheel.currentRPM);
        telemetry.addData("Flywheel 2 RPM", robot.flywheel.currentRPM2);

        telemetry.addData("Shooter 1 ", robot.flywheel.shooter1Speed);
        telemetry.addData("Shooter 2", robot.flywheel.shooter2Speed);

        telemetry.addData("toggle", robot.previousPrimeShooter);

        //telemetry.addData("leftmemer", (robot.robotPose.C[0][0] + " " + robot.robotPose.C[0][1] + " " + robot.robotPose.C[0][2] + " " + robot.robotPose.C[1][0] + " " + robot.robotPose.C[1][1] + " " + robot.robotPose.C[1][2] + " " + robot.robotPose.C[2][0] + " " + robot.robotPose.C[2][1] + " " + robot.robotPose.C[2][2]));
        //telemetry.addData("leftmemer", (robot.robotPose.CInverse[0][0] + " " + robot.robotPose.CInverse[0][1] + " " + robot.robotPose.CInverse[0][2] + " " + robot.robotPose.CInverse[1][0] + " " + robot.robotPose.CInverse[1][1] + " " + robot.robotPose.CInverse[1][2] + " " + robot.robotPose.CInverse[2][0] + " " + robot.robotPose.CInverse[2][1] + " " + robot.robotPose.CInverse[2][2]));
        //telemetry.addData("leftmemer", (robot.robotPose.deltaThetas[1][0] + " " + robot.robotPose.deltaThetas[1][0] + " " + robot.robotPose.deltaThetas[2][0]));
        //telemetry.addData("leftmemer", (robot.robotPose.encoderTicks[0][0] + " " + robot.robotPose.encoderTicks[1][0] + " " + robot.robotPose.encoderTicks[2][0]));
    }
}