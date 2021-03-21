package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.autos.ThreePowerShot;
import org.firstinspires.ftc.teamcode.drive.opmode.TurnTest;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Util;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.ml.RTrees;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.openCV.*;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PERPINDICULAR;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PARALLEL;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.WHEEL_RADIUS;
import static org.opencv.imgproc.Imgproc.moments;


@TeleOp(name = "wobbleDetection", group = "tests")
public class wobbleDetection extends LinearOpMode {


    private DcMotor RightForward, LeftForward, RightBack, LeftBack;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static String SkyStonePos;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 1f / 20f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    //red - 185
    //blue - 118
    private static final int threshold = 180;

    private static int value;

    OpenCvCamera webcam;

    StageSwitchingPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException{


        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");

        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftForward.setDirection(DcMotor.Direction.REVERSE);
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);


        webcam.openCameraDevice();//open camera
        pipeline = new StageSwitchingPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Waiting for start");

        while(!isStopRequested() && !isStarted()){
            telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.update();
        }


        if (opModeIsActive() && !isStopRequested()) {



            while (value < 145) {

                value = pipeline.getAnalysis();

                RightForward.setPower(0.2);
                RightBack.setPower(0.2);
                LeftForward.setPower(-0.2);
                LeftBack.setPower(-0.2);

                telemetry.addData("Values", value);
                telemetry.update();

            }

                RightForward.setPower(0);
                RightBack.setPower(0);
                LeftForward.setPower(0);
                LeftBack.setPower(0);


            telemetry.addData("Values", value);
            telemetry.update();



        }

    }
    public static class StageSwitchingPipeline extends OpenCvPipeline
    {

        public enum wobbleState {
            detected, notPresent
        }

        /*
         * Some color constants
         */

        static final Scalar BLACK = new Scalar(0, 0, 0);
        static final Scalar ORANGE = new Scalar(255, 110, 2);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(330,325);

        static final int REGION_WIDTH = 95;
        static final int REGION_HEIGHT = 90;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 134;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile StageSwitchingPipeline.wobbleState presence = StageSwitchingPipeline.wobbleState.notPresent;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCr(firstFrame);

            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCr(input);

            avg1 = (int) Core.mean(region1_Cr).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLACK, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            presence = wobbleState.notPresent; // Record our configuration

            if(avg1 > FOUR_RING_THRESHOLD){
                presence = wobbleState.detected;
            }else{
                presence = wobbleState.notPresent;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    5);

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}




