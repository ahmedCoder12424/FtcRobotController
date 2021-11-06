package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp(name="AutonTest", group="Auton Opmode")
public class AutonTest extends OpMode {

    Robot robot;
    boolean first = true;
    long timeChange = 0;
    long lastTime = System.currentTimeMillis();

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    private int ringCount = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.mpController.updateRequestedPose(0.00001, 0, 0, 0, 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

    }

    @Override
    public void init_loop(){

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);

        telemetry.addData("getP", robot.mpController.getP);
        telemetry.addData("xCor", robot.xCor);
        telemetry.addData("yCor", robot.yCor);
        telemetry.addData("rCor", robot.rCor);
        telemetry.addData("tDelta", (System.currentTimeMillis() - timeChange - robot.mpController.initTime));
        telemetry.addData("getVX", (robot.mpController.motionProfileX.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPX", (robot.mpController.motionProfileX.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getVY", (robot.mpController.motionProfileY.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPY", (robot.mpController.motionProfileY.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));


        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());

        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            ringCount = 4;
        } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            ringCount = 1;
        } else {
            ringCount = 0;
        }

        telemetry.addData("Ring Count", ringCount);

        telemetry.addData("error", robot.mpController.pidTheta.error);
    }

    @Override
    public void loop() {

        robot.mpController.updateRequestedPose(0, -20, 0, 0, 0);


        robot.updateLoop();
        if (first) {
            timeChange = System.currentTimeMillis() - robot.mpController.initTime;
            first = false;
        }

        telemetry.addData("dTime", System.currentTimeMillis() - lastTime);
        lastTime = System.currentTimeMillis();
        telemetry.addData("getP", robot.mpController.getP);
        telemetry.addData("xCor", robot.xCor);
        telemetry.addData("yCor", robot.yCor);
        telemetry.addData("rCor", robot.rCor);
        telemetry.addData("tDelta", (System.currentTimeMillis() - timeChange - robot.mpController.initTime));
        telemetry.addData("getVX", (robot.mpController.motionProfileX.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPX", (robot.mpController.motionProfileX.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getVY", (robot.mpController.motionProfileY.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPY", (robot.mpController.motionProfileY.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));


        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);

        telemetry.addData("Ring Count", ringCount);
        telemetry.addData("error", robot.mpController.pidTheta.error);

        /* --Telemetry--
        telemetry.addData("stopped", robot.stopped(true));
        telemetry.addData("PositionY", robot.currentY);
        telemetry.addData("PositionTargetY", robot.targetY);
        telemetry.addData("CorrectionY", robot.correctionY);
        telemetry.addData("highestY", robot.highestY);
        telemetry.addData("PositionX", robot.currentX);
        telemetry.addData("PositionTargetX", robot.targetX);
        telemetry.addData("CorrectionX", robot.correctionX);
        telemetry.addData("highestX", robot.highestX);
        telemetry.addData("PositionTargetX", robot.targetX);
        telemetry.addData("Rotation", robot.currentR);
        telemetry.addData("RotationTarget", robot.targetR);
        telemetry.addData("CorrectionR", robot.correctionR);
         */
    }

    private static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        //CHANGE TO TUNE
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90,90);

        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 50;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}
