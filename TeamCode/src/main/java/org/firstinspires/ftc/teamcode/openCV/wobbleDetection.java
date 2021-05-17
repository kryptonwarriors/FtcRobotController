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

import org.firstinspires.ftc.teamcode.drive.opmode.TurnTest;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Util;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.opencv.core.*;
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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        pipeline = new StageSwitchingPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Waiting for change");
        telemetry.update();

        sleep(1000);


        /*while(!isStopRequested() && !isStarted()){
            telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.update();
        }
*/




        if (opModeIsActive() && !isStopRequested()) {



        }

    }
    public static class StageSwitchingPipeline extends OpenCvPipeline
    {

        /*
         * Working variables
         */
        Mat mat;
        Mat ret;
        Height height;


        public enum Height {
            wobble, noWobble
    }

    Scalar lowerOrange = new Scalar(0, 141, 0);
    Scalar upperOrange = new Scalar(255, 230, 95);

    int CAMERA_WIDTH = 320;

    int HORIZON = (int) (100/320) * CAMERA_WIDTH;

    int MID_WIDTH = (50/320) * CAMERA_WIDTH;

    static double BOUND_RATIO = 0.7;

        @Override
        public void init(Mat firstFrame)
        {
            height = Height.noWobble;
            ret = new Mat();
            mat = new Mat();
        }

        @Override
        public Mat processFrame(Mat input)
        {
            ret.release();

            ret = new Mat();

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            Core.inRange(mat, lowerOrange, upperOrange, mask);

            Core.bitwise_and(input, input, ret, mask);

            Imgproc.GaussianBlur(mask, mask, new Size(5, 15), 0);

            List<MatOfPoint> countours = new ArrayList();

            Mat hierarchy = new Mat();

          //  Imgproc.findContours(mask)


            return input;
        }

    }

}




