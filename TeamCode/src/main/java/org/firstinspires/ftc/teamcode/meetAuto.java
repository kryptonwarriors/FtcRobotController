package org.firstinspires.ftc.teamcode;

// import Rahul's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
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

import org.firstinspires.ftc.teamcode.util.Util;
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

    //Variabliity Variables
    boolean turnToDrop = false;
    double distanceToStrafeToZone;

    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    private Orientation angles;
    public Orientation lastAngles = new Orientation();
    PIDController drive;
    PIDController strafe;

    private final int rows = 640;
    private final int cols = 480;
    OpenCvCamera webcam;
    RingDeterminationPipeline pipeline;

    public ElapsedTime runtime = new ElapsedTime();
    public Util util = new Util(runtime, hardwareMap );
    public AutoAlign align = new AutoAlign();


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
*/
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");

        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        //Initialization
        while (!(isStopRequested() || isStarted())) {

            //Update Neccessary Variables depending on Ring Configuration
            if (pipeline.position == RingDeterminationPipeline.RingConfiguration.C) {

            } else if (pipeline.position == RingDeterminationPipeline.RingConfiguration.B) {
                turnToDrop = true;
            } else if (pipeline.position == RingDeterminationPipeline.RingConfiguration.A){
                distanceToStrafeToZone = 30;
            }

            telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.addData("ringConfig", pipeline.position);
            telemetry.addData("distanceToStrafe", distanceToStrafeToZone);
            telemetry.addData(">>>", "INITIALIZATION COMPLETED");
            telemetry.update();

            sleep(50);
        }

        while (opModeIsActive()) {

            //1.Go Forward A little
                moveDistance(FORWARD, 0.7, 15, 1000);

            //2. Rotate

                LeftForward.setPower(0.5);
                LeftBack.setPower(0.5);
                RightForward.setPower(0.5);
                RightBack.setPower(0.5);

                while (getAngle() > -72) {
                    telemetry.addData("angle", getAngle());
                    telemetry.update();
                }
                util.StopDrive();

            //3. Backward to Wall
                moveDistance(BACKWARD, 0.5, 4, 500);

            //4. Strafe to Zone
                moveDistance(RIGHT, 0.7, distanceToStrafeToZone, 500000000);

            //4.5 Rotate 180 degrees if needed to drop Wobble Goal in Zone B
                if (turnToDrop) {
                    //Rotate 180 with IMU
                }

            //5. Drop Goal

            //6. Go To Launch Line
                moveDistance(LEFT, 0.7, align.inchesToVerticalAlignment, 500000000);

            //7. Align For Shooting

            //8. Shoot

        }
    }

    //END


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

    public void moveDistance(int Direction, double Power, double distance, int safetyTargetPosition){
        if (Direction == FORWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) < distance) {
                correction = drive.performPID(getAngle());

                LeftForward.setPower(Power + correction);
                LeftBack.setPower(Power + correction);
                RightForward.setPower(Power - correction);
                RightBack.setPower(Power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == BACKWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) > distance) {
                correction = drive.performPID(getAngle());

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == RIGHT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > distance) {
                correction = strafe.performPID(getAngle());

                LeftForward.setPower(-Power + correction);
                LeftBack.setPower(Power + correction);
                RightForward.setPower(Power - correction);
                RightBack.setPower(-Power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == LEFT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) < distance) {
                correction = strafe.performPID(getAngle());

                LeftForward.setPower(Power + correction);
                LeftBack.setPower(-Power + correction);
                RightForward.setPower(-Power - correction);
                RightBack.setPower(Power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        util.StopDrive();

    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring position
         */
        public enum RingConfiguration
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200,280);

        static final int REGION_WIDTH = 90;
        static final int REGION_HEIGHT = 55;

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
        private volatile RingConfiguration position = RingConfiguration.C;

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

            position = RingConfiguration.C; // Record our configuration
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingConfiguration.C;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingConfiguration.B;
            }else{
                position = RingConfiguration.A;
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

    private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -90)
            deltaAngle += 360;
        else if (deltaAngle > 90)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}