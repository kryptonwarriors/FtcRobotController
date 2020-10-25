package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;


import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RedPracticeAuto", group = "")
public class RedPracticeAuto extends LinearOpMode {

    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer;
    public Servo WobbleClamper, RingClamper;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;

    private static float rectHeight = .6f / 8f;


    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255


    private static int valBottom = -1;
    private static int valTop = -1;
    private static String ringConfig;
    private static float rectWidth = 1.5f / 7f;
    private static float offsetX = -0.3f / 8f;
    //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = -.4f / 8f;
    //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] bottomPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] topPos = {4f / 8f + offsetX, 3.42f / 8f + offsetY};

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        Wobbler = hardwareMap.dcMotor.get("Wobbler");
        Ringer = hardwareMap.dcMotor.get("Ringer");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");
        RingClamper = hardwareMap.servo.get("RingClamper");


        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        WobbleClamper.setPosition(0.25);
        RingClamper.setPosition(0.1);

        runtime.reset();

        while (!(isStopRequested() || isStarted())) {

            if (valBottom != 0) {
                if (valTop != 0) {
                    ringConfig = "C";
                } else {
                    ringConfig = "B";
                }
            } else {
                ringConfig = "A";
            }


            telemetry.addData("ringConfig", ringConfig);
            //telemetry.addData ( "Height", rows );
            //telemetry.addData ( "Width", cols );

            telemetry.update();
        }

        if (opModeIsActive()) {


            if (ringConfig == "A") {
                //Go To Wobble Goal Drop Zone
                moveEncoders(FORWARD, 0.7, 2200);
               moveEncoders(LTURN, 0.6, 650);
               //Drop Wobble Goal
               Wobbler.setPower(-1);
               WobbleClamper.setPosition(0);
               sleep(500);
               Wobbler.setPower(0);
               //Go to Low Goal
               moveEncoders(RIGHT, 0.5, 1200);
               moveEncoders(LTURN, 0.6, 650);
               moveEncoders(RIGHT, 0.5, 600);

               //Drop Rings into Goal
               Ringer.setPower(-1);
               sleep(1000);
               Ringer.setPower(0);
               RingClamper.setPosition(0.6);

            } else if (ringConfig == "B") {
                //Go To Wobble Drop Zone
                moveEncoders(RIGHT, 0.5, 700);
                moveEncoders(FORWARD, 0.7, 2200);
                moveEncoders(RTURN, 0.6, 650);
                //Drop Wobble Goal
                Wobbler.setPower(-1);
                WobbleClamper.setPosition(0);
                sleep(500);
                Wobbler.setPower(0);

            } else if (ringConfig == "C") {
                //Go to Wobble Drop Zone
                moveEncoders(RIGHT, 0.5, 700);
                moveEncoders(FORWARD, 0.7, 2200);
                moveEncoders(RTURN, 0.6, 1300);
                //Drop Wobble Goal
                Wobbler.setPower(-1);
                WobbleClamper.setPosition(0);
                sleep(500);
                Wobbler.setPower(0);

            }

            telemetry.addData("Status", "Program Complete");
            telemetry.addData("Position", LeftForward.getCurrentPosition() + "," + LeftForward.getTargetPosition());
            telemetry.update();

        }
    }



    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat,
                    Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST,
                    Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixBottom = thresholdMat.get((int) (input.rows() * bottomPos[1]),
                    (int) (input.cols() * bottomPos[0]));//gets value at circle
            valBottom = (int) pixBottom[0];

            double[] pixTop = thresholdMat.get((int) (input.rows() * topPos[1]),
                    (int) (input.cols() * topPos[0]));//gets value at circle
            valTop = (int) pixTop[0];


            //create three points
            Point pointBottom = new Point((int) (input.cols() * bottomPos[0]),
                    (int) (input.rows() * bottomPos[1]));
            Point pointTop = new Point((int) (input.cols() * topPos[0]),
                    (int) (input.rows() * topPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointBottom, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointTop, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (topPos[0] - rectWidth / 2),
                            input.rows() * (topPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (topPos[0] + rectWidth / 2),
                            input.rows() * (topPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (bottomPos[0] - rectWidth / 2),
                            input.rows() * (bottomPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (bottomPos[0] + rectWidth / 2),
                            input.rows() * (bottomPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }


    public void moveEncoders(int Direction, double Power, int TargetPosition) {
        LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if (Direction == FORWARD) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                LeftBack.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == BACKWARD) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(-TargetPosition);
                LeftBack.setTargetPosition(-TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == LEFT) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                LeftBack.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == RIGHT) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(-TargetPosition);
                LeftBack.setTargetPosition(-TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == LTURN) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                LeftBack.setTargetPosition(-TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == RTURN) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {
                LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(-TargetPosition);
                LeftBack.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        }
        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

    }
}


