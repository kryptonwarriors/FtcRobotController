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
import org.opencv.core.MatOfPoint2f;
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


@TeleOp(name = "colorReact", group = "tests")
public class colorReact extends LinearOpMode {


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

    public final int rows = 640;
    public final int cols = 480;


    //red - 185
    //blue - 118
    public static final int threshold = 180;



    OpenCvCamera webcam;

    StageSwitchingPipeline pipeline;
    
    @Override
    public void runOpMode() throws InterruptedException{


        /*RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");

        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftForward.setDirection(DcMotor.Direction.REVERSE);
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        pipeline = new StageSwitchingPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        while (!isStopRequested()) {

            telemetry.addData("width", pipeline.getWidth());
            telemetry.addData("x", pipeline.getXPos());
            telemetry.addData("y", pipeline.getYPos());
            telemetry.addData("height", pipeline.getHeight());
            telemetry.addData("area", pipeline.getArea());
            telemetry.update();

        }

    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        static final Scalar ORANGE = new Scalar(255, 110, 2);

        public int width;
        public int x;
        public int y;
        public int height;
        public double area;

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();




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

            //color diff cr.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);;//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 1);//takes cr difference and stores (coi is channel of interest)
                                                                     //0 = Y, 1 = Cr, 2 = Cb

            //b&w (thresholding to make a map of the desired color
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, threshold, 255, Imgproc.THRESH_BINARY);

            Mat eroder= Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size( 3, 3));

            Mat dilator = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8,8));

            Imgproc.erode(thresholdMat, thresholdMat, eroder);
            Imgproc.erode(thresholdMat, thresholdMat, eroder);

            Imgproc.dilate(thresholdMat, thresholdMat, dilator);
            Imgproc.dilate(thresholdMat, thresholdMat, dilator);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
            yCbCrChan2Mat.copyTo(all);
            Imgproc.drawContours(all, contoursList, -1, ORANGE, 4, 8);

            int maxWidth = 0;
            int tempX = 0;

            Rect maxRect = new Rect();

            for(MatOfPoint c : contoursList ){
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                int w = rect.width;
                int x = rect.x;

                if(w > maxWidth){
                    maxWidth = w;
                    maxRect = rect;
                    tempX = x;
                }

                c.release();
                copy.release();
            }

            Imgproc.rectangle(thresholdMat, maxRect, new Scalar(0, 0.0, 255), 10);
            Imgproc.rectangle(input, maxRect, new Scalar(0, 0.0, 255), 10);
            Imgproc.rectangle(all, maxRect, new Scalar(0, 0.0, 255), 10);

           // Imgproc.line(input, new Point(50,0), new Point(50, 480), new Scalar(255, 110, 2), 2);
           // Imgproc.line(all, new Point(50,0), new Point(50, 480), new Scalar(255, 110, 2), 2);


            width = maxRect.width;
            x = maxRect.x;
            y = maxRect.y;
            area = maxRect.area();
            height = maxRect.height;

                switch (stageToRenderToViewport) {
                    case THRESHOLD: {
                        return thresholdMat;
                    }

                    case detection:{
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

        public int getWidth() {return width;}
        public int getXPos() {return x;}
        public int getYPos() {return y;}
        public int getHeight() {return height;}
        public double getArea() {return area;}

    }

}




