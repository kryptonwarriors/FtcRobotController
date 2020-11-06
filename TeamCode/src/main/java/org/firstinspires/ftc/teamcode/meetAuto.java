package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;

import org.firstinspires.ftc.teamcode.AutoAlign;


import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;

@Autonomous(name = "meetAuto", group = "")
public class meetAuto extends LinearOpMode {
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer;
    public Servo WobbleClamper, RingClamper;
    public DistanceSensor BackDistance, RightDistance;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;

    boolean initialStrafeRight = false;
    double distanceToStrafe;

    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    private Orientation angles;
    public Orientation lastAngle = new Orientation();

    private final int rows = 640;
    private final int cols = 480;
    OpenCvCamera webcam;
    RingDeterminationPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");
       /*
        Wobbler = hardwareMap.dcMotor.get("Wobbler");
        Ringer = hardwareMap.dcMotor.get("Ringer");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");
        RingClamper = hardwareMap.servo.get("RingClamper");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
*/
        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new RingDeterminationPipeline());//different stages
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(rows, cols);
            }
        });

        //Initialization
        while (!(isStopRequested() || isStarted())) {

            if (pipeline.position == RingDeterminationPipeline.RingPosition.C) {

            } else if (pipeline.position == RingDeterminationPipeline.RingPosition.B) {
                initialStrafeRight = true;
            } else if (pipeline.position == RingDeterminationPipeline.RingPosition.A){
                distanceToStrafe = 30;
            }


                telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.addData("ringConfig", pipeline.position);
            //telemetry.addData ( "Height", rows );
            //telemetry.addData ( "Width", cols );

            telemetry.update();
        }

        while (opModeIsActive()) {

        }
    }
    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring position
         */
        public enum RingPosition
        {
            C,
            B,
            A
        }

        /*
         * Some color constants
         */
        static final Scalar BLACK = new Scalar(0, 0, 0);
        static final Scalar ORANGE = new Scalar(255, 110, 2);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(262,196);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

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
        private volatile RingPosition position = RingPosition.C;

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
                    BLACK, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.C; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.C;
            } else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.B;
            } else{
                position = RingPosition.A;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}