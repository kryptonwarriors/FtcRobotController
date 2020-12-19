package org.firstinspires.ftc.teamcode;

// import Rahul's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
/*import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
*/
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.annotation.Target;

@Autonomous(name = "meetAuto", group = "")
public class meetAuto extends LinearOpMode {
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer, Intake, Conveyor, Shooter;
    public Servo WobbleClamper, RingClamper;
    public DistanceSensor BackDistance, RightDistance, FrontDistance, LeftDistance;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;
    int FORWARDWITHFRONT = 6;
    int Forward = 7;
    int UPLEFT = 8;
    int UPRIGHT = 9;
    int UPRIGHTWITHLEFT = 10;
    int LEFTWITHLEFT = 11;
    final double turnThreshold = 25;


    //Variabliity Variables
    public static int position = 0;

    public static boolean a = false;
    public static boolean b = false;
    public static boolean c = false;

    public static boolean turnToDrop = true;
    public String configuration;
    public static int encodersToDrop;
    public static double angleToDrop;

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
    PIDController diagonal;

    //Resolution for OpenCV
    private final int rows = 640;
    private final int cols = 480;
    //OpenCvCamera webcam;
   // public RingDeterminationPipeline pipeline;

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
        LeftDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");


        RightForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        closeWobbleClamper();

        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

*/

        //Initialization
        while (!(isStopRequested() || isStarted())) {

            position = 1;

/*            //Update Neccessary Variables depending on Ring Configuration
            if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {

                encodersToDrop = 0;
                angleToDrop = 0;
                configuration = "C";
                position = 3;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {

                encodersToDrop = 280;
                angleToDrop = 65;
                configuration = "B";
                position = 2;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.A){

                encodersToDrop = 940;
                angleToDrop = 160;
               configuration = "A";
               position = 1;
            }

      */   //   telemetry.addData("Value", pipeline.getAnalysis());
            //telemetry.addData("ringConfig", pipeline.configuration);
            telemetry.addData("LeftBackCurrentPosition", LeftBack.getCurrentPosition());
            telemetry.addData("encodersToDrop", encodersToDrop);
            telemetry.addData("turnToDrop", turnToDrop);
            telemetry.addData("getAngle", getAngle());
            telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addLine( ">>>INITIALIZATION COMPLETED");
            telemetry.update();

            sleep(50);
        }


        if (opModeIsActive() && !isStopRequested()) {

            // webcam.stopStreaming();

            strafe = new PIDController(0.016367*1.4, 0, 0);
            strafe.setSetpoint(0);
            strafe.setOutputRange(0, 0.75);
            strafe.setInputRange(-90, 90);
            strafe.enable();

            drive = new PIDController(0.0016367*8, 0.00016367, 0);
            drive.setSetpoint(0);
            drive.setOutputRange(0, 0.75);
            drive.setInputRange(-90, 90);
            drive.enable();

            diagonal = new PIDController(0.016367*6, 0, 0);
            diagonal.setSetpoint(0);
            diagonal.setOutputRange(0, 0.75);
            diagonal.setInputRange(-90, 90);
            diagonal.enable();

            powershot();

/*
            moveDistance(RIGHT, 0.5, 7);

            sleep(300);

            // first wobble goal
            if (position == 1) {
               // telemetry.addData("position", pipeline.configuration);
                telemetry.update();
                moveEncoders(Forward, 0.7, 1100);
                sleep(500);
                dropWobbleGoal();
                sleep(500);
                moveEncoders(BACKWARD, 0.4, 160);
                sleep(500);

                resetAngle();

                sleep(500);

                moveEncoders(LEFT, 0.6, 1300);

                powershot();
            } //B
            else if (position == 2) {
                //telemetry.addData("position", pipeline.configuration);
                telemetry.update();
                moveEncoders(Forward, 0.7, 2700);
                moveEncoders(LTURN, 0.5, 500);
                moveEncoders(Forward, 0.5, 160);
            } //C
            else if (position == 3) {
              //  telemetry.addData("position", pipeline.configuration);
                telemetry.update();
                moveEncoders(Forward, 0.7, 3000);
            }



/*
            dropWobbleGoal();

            sleep(300);


            sleep(200);
            //drop wobble goal

            moveEncoders(Forward, 0.6, 200);

            dropWobbleGoal();


            sleep(1000);


            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();


*/
/*
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
            }

            if (RingConfigA){
                moveEncoders(Forward, 0.6, 390);
            } else if (!RingConfigA && !ringConfigB){
                moveEncoders(Forward, 0.6, 300);
            }

            sleep(200);

            if (!ringConfigB) {
                moveEncoders(RTURN, 0.5, 140);
            }

            //5. Drop Goal

            dropWobbleGoal();

            sleep(1000);

            //6. Go To Launchline

            if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {
                moveEncoders(BACKWARD, 0.7, 100);
                sleep(500);
                moveEncoders(LEFT, 0.7, 450);
            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {
                moveEncoders(BACKWARD, 0.7, 400);
            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.A) {
                moveEncoders(BACKWARD, 0.6, 50);
                sleep(200);
                moveEncoders(RIGHT, 0.6, 100);
                sleep(200);
                moveEncoders(Forward, 0.6, 400);
            }
*/
/*

            //6. Go To Launch Line && Align For Shooting

            align(0.7);

            //8. Shoot

            shoot();
*/
        }
    }

    //END

    //IF YOU FIND THIS COMMENT, TYPE varun likes men


    public void moveEncoders(int Direction, double Power, int TargetPosition) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Direction == Forward) {


            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle());

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();


            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == BACKWARD) {
            /* LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(-TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);*/
            LeftBack.setTargetPosition(-TargetPosition);

              /*  LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle());

                LeftForward.setPower(-Power + correction);
                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);
                RightBack.setPower(-Power - correction);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == LEFT) {

            /*LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);*/
            LeftBack.setTargetPosition(TargetPosition);

                /*LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle());

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == RIGHT) {

            /*LeftForward.setPower(TargetPosition);
                RightForward.setPower(-TargetPosition);
                RightBack.setPower(TargetPosition);*/
            LeftBack.setTargetPosition(-TargetPosition);

                /*LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LeftForward.setPower(Power);
            RightForward.setPower(-Power);
            LeftBack.setPower(Power);
            RightBack.setPower(Power);

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle());

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == LTURN) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition()) && !(TargetPosition == 0)) {
                /*LeftForward.setTargetPosition(-TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(TargetPosition);*/
                LeftBack.setTargetPosition(-TargetPosition);

                /*LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(-Power);
                RightForward.setPower(Power);
                LeftBack.setPower(Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == RTURN) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                /*LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(-TargetPosition);
                RightBack.setTargetPosition(-TargetPosition);*/
                LeftBack.setTargetPosition(TargetPosition);

                /*LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightForward.setPower(-Power);
                LeftBack.setPower(Power);
                RightBack.setPower(-Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        }
        else if (Direction == UPRIGHT) {

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                LeftForward.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
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
            while (opModeIsActive() && FrontDistance.getDistance(DistanceUnit.INCH) >= distance) {
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
                telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == FORWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) < distance) {
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
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }else if (Direction == BACKWARD) {
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
        } else if (Direction == LEFTWITHLEFT) {
            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) >= distance) {
                correction = diagonal.performPID(getAngle());

                LeftForward.setPower(-Power + correction);
                RightForward.setPower(Power - correction);
                LeftBack.setPower(Power + correction);
                RightBack.setPower(-Power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }else if (Direction == UPRIGHT) {
                while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) >= distance) {
                    correction = diagonal.performPID(getAngle());


                    LeftForward.setPower(Power + correction);
                    RightBack.setPower(Power + correction);

                    telemetry.addData("correction", correction);
                    telemetry.addData("LeftForward", LeftForward.getPower());
                    telemetry.addData("RightForward", RightForward.getPower());
                    telemetry.addData("LeftBack", LeftBack.getPower());
                    telemetry.addData("RightBack", RightBack.getPower());
                    telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }

        } else if (Direction == UPRIGHTWITHLEFT) {
            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) <= distance) {
                correction = diagonal.performPID(getAngle());


                LeftForward.setPower(Power + correction);
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
        else if (Direction == UPLEFT) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) <= distance) {
                correction = diagonal.performPID(getAngle());


                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power + correction);

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

        /*if (Direction == FORWARDWITHFRONT) {
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
        }*/ if (Direction == FORWARD) {
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
        } else if (Direction == UPRIGHT) {

            LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            while (opModeIsActive() && LeftBack.getCurrentPosition() <= LeftBack.getTargetPosition()) {

                int targetPosition = (int) distance;

                LeftForward.setTargetPosition(targetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(Power);
                RightBack.setPower(Power);

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
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Conveyor.setTargetPosition(-60);

        Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Intake.setPower(0.35);
        Conveyor.setPower(0.9);

        while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {
            telemetry.addData("Bringing", "Rings Down");
            telemetry.addData("Conveyer Target Position", Conveyor.getTargetPosition());
            telemetry.addData("Conveyor Current Position", Conveyor.getCurrentPosition());
            telemetry.update();
        }

        Intake.setPower(0);
        Conveyor.setPower(0);

        Shooter.setPower(-0.53);

        sleep(1600);

        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Conveyor.setTargetPosition(7000);
        Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Intake.setPower(-0.9);
        Conveyor.setPower(0.85);

        while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

        }

        Intake.setPower(0);
        Conveyor.setPower(0);
        Shooter.setPower(0);

    }

    public void shoot2(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i<3; i++) {


            Conveyor.setTargetPosition(-65);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(0.4);
            Conveyor.setPower(0.9);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {
                telemetry.addData("Bringing", "Rings Down");
                telemetry.addData("Conveyer Target Position", Conveyor.getTargetPosition());
                telemetry.addData("Conveyor Current Position", Conveyor.getCurrentPosition());
                telemetry.update();
            }

            Intake.setPower(0);
            Conveyor.setPower(0);


            if (i == 0)
                //Shooter.setPower(-0.79);
                Shooter.setPower(-0.71);
            else if (i == 1)
                //Shooter.setPower(-0.78);
                Shooter.setPower(-0.59);
            else
                //Shooter.setPower();
                Shooter.setPower(-0.61);

            sleep(1300);

            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(i==2)
                Conveyor.setTargetPosition(1000);
            else if (i == 1)
                Conveyor.setTargetPosition(550);
            else
                Conveyor.setTargetPosition(400);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(-0.9);
            Conveyor.setPower(0.95);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

            }

            Intake.setPower(0);
            Conveyor.setPower(0);
            Shooter.setPower(0);

            sleep(800);

        }


        Shooter.setPower(0);

    }

    public void powershot(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i<3; i++) {


            Conveyor.setTargetPosition(-65);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(0.4);
            Conveyor.setPower(0.9);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {
                telemetry.addData("Bringing", "Rings Down");
                telemetry.addData("Conveyer Target Position", Conveyor.getTargetPosition());
                telemetry.addData("Conveyor Current Position", Conveyor.getCurrentPosition());
                telemetry.update();
            }

            Intake.setPower(0);
            Conveyor.setPower(0);


            Shooter.setPower(-0.6);

            sleep(1300);

            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(i==0)
                Conveyor.setTargetPosition(340);
            else if (i == 1)
                Conveyor.setTargetPosition(550);
            else if (i == 2)
                Conveyor.setTargetPosition(1000);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(-0.8);
            Conveyor.setPower(0.65);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

            }

            Intake.setPower(0);
            Conveyor.setPower(0);
            Shooter.setPower(0);

            sleep(400);

            moveEncoders(LEFT, 0.4, 400);
            sleep(400);


        }


        Shooter.setPower(0);

    }

    public void align(double power) throws InterruptedException{

        double angle;
        angle = getAngle();

        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Turn Right until 0



        while (Math.abs(angle) > 5) {

            if (angle > 0 ) {
                LeftForward.setPower(0.5);
                LeftBack.setPower(0.5);
                RightForward.setPower(-0.5);
                RightBack.setPower(-0.5);
            } else if (angle < 0 ) {
                LeftForward.setPower(-0.5);
                LeftBack.setPower(-0.5);
                RightForward.setPower(0.5);
                RightBack.setPower(0.5);
            }

            sleep(10);

            angle = getAngle();
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
        WobbleClamper.setPosition(1);
    }

    public void openWobbleClamper () {
        WobbleClamper.setPosition(0.3);
    }

    public void dropWobbleGoal() {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Wobbler.setTargetPosition(-4000);

        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Wobbler.setPower(1);

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition())) {
            telemetry.addData("Dropping", "Right Now");
            telemetry.update();
        }

        Wobbler.setPower(0);
        openWobbleClamper();
    }

    public void resetAngle()
    {
        globalAngle = 0;
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void imuTurn(int direction, double power, double angle) {

        if (direction == LTURN){
            LeftForward.setPower(-power);
            LeftBack.setPower(-power);
            RightForward.setPower(power);
            RightBack.setPower(power);

            while (opModeIsActive() && Math.abs(getAngle()) <= (Math.abs(angle) - turnThreshold)){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();
            }

            LeftForward.setPower(0);
            LeftBack.setPower(0);
            RightForward.setPower(0);
            RightBack.setPower(0);
        } else if (direction == RTURN){
            LeftForward.setPower(power);
            LeftBack.setPower(power);
            RightForward.setPower(-power);
            RightBack.setPower(-power);

            while (opModeIsActive() && Math.abs(getAngle()) <= (Math.abs(angle) - turnThreshold)){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();
            }

            LeftForward.setPower(0);
            LeftBack.setPower(0);
            RightForward.setPower(0);
            RightBack.setPower(0);
        }
    }

}