package org.firstinspires.ftc.teamcode;

// import Rahul's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "meetAuto", group = "")
public class meetAuto extends LinearOpMode {
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer, Intake, Conveyor, Shooter;
    public Servo WobbleClamper, RingClamper;
    public DistanceSensor BackDistance, RightDistance, FrontDistance;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;
    int FORWARDWITHFRONT = 6;
    int Forward = 7;


    //Variabliity Variables
    boolean turnToDrop = true;
    boolean ringConfigB;
    int encodersToDrop;

    //Align Variables
    public double inchesToVerticalAlignment = 64;
    public double inchesToHorizontalAlignment = 31;

    // PID/IMU Variables
    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    private Orientation angles;
    public Orientation lastAngles = new Orientation();
    PIDController drive;
    PIDController strafe;

    //Resolution for OpenCV
    private final int rows = 640;
    private final int cols = 480;
    OpenCvCamera webcam;
    RingDeterminationPipeline pipeline;

    public ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");


        Intake = hardwareMap.dcMotor.get("Intake");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Shooter = hardwareMap.dcMotor.get("Shooter");

        Wobbler = hardwareMap.dcMotor.get("Wobbler");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        FrontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");

        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        closeWobbleClamper();


        //Initialization
        while (!(isStopRequested() || isStarted())) {

            //Update Neccessary Variables depending on Ring Configuration
            if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {

                turnToDrop = false;
                ringConfigB = false;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {

                encodersToDrop = 300;
                ringConfigB = true;
                turnToDrop = true;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.A){

                encodersToDrop = 1000;
                ringConfigB = false;
                turnToDrop = true;
            }

            telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.addData("ringConfig", pipeline.configuration);
            telemetry.addData("encodersToDrop", encodersToDrop);
            telemetry.addData("turnToDrop", turnToDrop);
            telemetry.addData(">>>", "INITIALIZATION COMPLETED");
            telemetry.update();

            sleep(50);
        }

        if (opModeIsActive() && !isStopRequested()) {

            strafe = new PIDController(0.0016367*2, 0.00016367, 0.000016367);
            strafe.setSetpoint(0);
            strafe.setOutputRange(0, 0.75);
            strafe.setInputRange(-90, 90);
            strafe.enable();

            drive = new PIDController(0.016367*2, 0.0016367*4, 0.00016367);
            drive.setSetpoint(0);
            drive.setOutputRange(0, 0.75);
            drive.setInputRange(-90, 90);
            drive.enable();

/*
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
*/



            //1.Go Forward A little
            moveDistanceWithOutPID(FORWARD, 0.4, 25);

            shoot();

            //2. Strafe Right To Wall
            moveDistance(RIGHT, 0.7, 12);

            resetAngle();
            sleep(1000);

            //3. Go To Middle of Target Zones
            moveEncoders(Forward, 0.7, 1800);

            sleep(800);

            //4. Rotate To Drop Wobble Goal
            if (turnToDrop) {
                moveEncoders(LTURN, 0.6, encodersToDrop);
                moveEncoders(Forward, 0.6, 100);
            }


            //TODO Adjust Config C and overall PID
            moveEncoders(RTURN, 0.6, 50);



            //5. Drop Goal

            dropWobbleGoal();

            sleep(1000);

            if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {
                moveEncoders(BACKWARD, 0.7, 300);
                sleep(500);
            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {
                moveEncoders(BACKWARD, 0.7, 400);
            }

/*

            //6. Go To Launch Line && Align For Shooting

            align(0.7);

            //8. Shoot

            shoot();
*/
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


        if (Direction == Forward) {

            while (!isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {

                correction = drive.performPID(getAngle());

                LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                LeftBack.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);
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

    public void moveDistance(int Direction, double Power, double distance){

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Direction == FORWARDWITHFRONT) {
            while (opModeIsActive() && FrontDistance.getDistance(DistanceUnit.INCH) > distance) {
                correction = drive.performPID(getAngle());

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } if (Direction == FORWARD) {
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
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }else if (Direction == BACKWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) > distance) {
                correction = drive.performPID(getAngle());

                LeftForward.setPower(-Power + correction);
                LeftBack.setPower(-Power + correction);
                RightForward.setPower(-Power - correction);
                RightBack.setPower(-Power - correction);

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

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

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

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
    }

    public void moveDistanceWithOutPID(int Direction, double Power, double distance){

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Direction == FORWARDWITHFRONT) {
            while (opModeIsActive() && FrontDistance.getDistance(DistanceUnit.INCH) > distance) {

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } if (Direction == FORWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) < distance) {

                LeftForward.setPower(Power);
                LeftBack.setPower(Power);
                RightForward.setPower(Power);
                RightBack.setPower(Power);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }else if (Direction == BACKWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) > distance) {
                correction = drive.performPID(getAngle());

                LeftForward.setPower(-Power + correction);
                LeftBack.setPower(-Power + correction);
                RightForward.setPower(-Power - correction);
                RightBack.setPower(-Power - correction);

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

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

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

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring configuration
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
        private volatile RingConfiguration configuration = RingConfiguration.C;

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

            configuration = RingConfiguration.C; // Record our configuration

            if(avg1 > FOUR_RING_THRESHOLD){
                configuration = RingConfiguration.C;
            }else if (avg1 > ONE_RING_THRESHOLD){
                configuration = RingConfiguration.B;
            }else{
                configuration = RingConfiguration.A;
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

    public void shoot(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Conveyor.setTargetPosition(200);

        Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Intake.setPower(-0.2);
        Conveyor.setPower(0.9);

        while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {
            telemetry.addData("Bringing", "Rings Down");
            telemetry.addData("Conveyer Target Position", Conveyor.getTargetPosition());
            telemetry.addData("Conveyor Current Position", Conveyor.getCurrentPosition());
            telemetry.update();
        }

        Intake.setPower(0);
        Conveyor.setPower(0);

        Shooter.setPower(-1);

        sleep(2000);

        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Conveyor.setTargetPosition(-2500);
        Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Intake.setPower(0.9);
        Conveyor.setPower(0.9);

        while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

        }

        Intake.setPower(0);
        Conveyor.setPower(0);
        Shooter.setPower(0);

    }

    public void align(double power) {
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Turn Right until 0

        LeftForward.setPower(0.5);
        LeftBack.setPower(0.5);
        RightForward.setPower(-0.5);
        RightBack.setPower(-0.5);

        while (Math.abs(getAngle()) > 5) {
            telemetry.addData("Action", "Turning");
            telemetry.update();
        }

        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

        //Move to Launch Line Vertically
        moveDistance(BACKWARD, 0.7, inchesToVerticalAlignment);

        //Align Horizontally for Shooting

        if (RightDistance.getDistance(DistanceUnit.INCH) < inchesToHorizontalAlignment) {
            moveDistance(LEFT, 0.7, inchesToHorizontalAlignment);
        } else if (RightDistance.getDistance(DistanceUnit.INCH) > inchesToHorizontalAlignment) {
            moveDistance(RIGHT, 0.7, inchesToHorizontalAlignment);
        }

        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }

    public void closeWobbleClamper() {
        WobbleClamper.setPosition(0.48);
    }

    public void openWobbleClamper () {
        WobbleClamper.setPosition(0.1);
    }

    public void dropWobbleGoal() {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Wobbler.setTargetPosition(-2400);

        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Wobbler.setPower(1);

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition())) {
            telemetry.addData("Dropping", "Right Now");
            telemetry.update();
        }

        Wobbler.setPower(0);
        openWobbleClamper();
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

}