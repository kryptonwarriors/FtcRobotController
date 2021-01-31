package org.firstinspires.ftc.teamcode.autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.TurnTest;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Util;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.ml.RTrees;
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

import java.lang.annotation.Target;

import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PERPINDICULAR;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PARALLEL;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.WHEEL_RADIUS;



@Autonomous(name = "ThreePowerShot", group = "autos")
public class ThreePowerShot extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer, Intake, Conveyor, Shooter;
    public Servo WobbleClamper, RingClamper;
    public DistanceSensor BackDistance, RightDistance, FrontDistance, LeftDistance;

    public OpenCvCamera webcam;
    public RingDeterminationPipeline pipeline;

    // PID/IMU Variables
    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    private Orientation angles;
    public Orientation lastAngles = new Orientation();
    PIDController drive;
    PIDController backDrive;
    PIDController strafe;
    PIDController diagonal;

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
    int TURNED_FORWARD = 12;
    int TURNED_BACKWARD = 13;
    int FORWARD_WITH_ARM = 14;
    int FORWARDWITHBACK = 15;
    int DISTANCELEFT = 16;
    int ENCODER_RIGHT = 17;
    int TURNED_RIGHT = 18;

    private VoltageSensor voltageSensor;
    private double initialVoltage;

    double angle;
    public static int diagonalDistance;
    public static int secondWobbleGoalDistance;
    public static int secondWobbleEncoderDistance;
    public static int position;
    public static int movement;
    public static int angleForSecond;

    final double turnThreshold = 25;

    public static int encodersToDrop;
    public static int angleToDrop;


    @Override
    public void runOpMode() throws InterruptedException{


        /* DRIVE MOTORS */
        LeftForward = hardwareMap.get ( DcMotor.class, "LeftForward" );
        RightForward = hardwareMap.get ( DcMotor.class, "RightForward" );
        LeftBack = hardwareMap.get ( DcMotor.class, "LeftBack" );
        RightBack = hardwareMap.get ( DcMotor.class, "RightBack" );

        /* SUBSYSTEM */
        Intake = hardwareMap.dcMotor.get("Intake");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Shooter = hardwareMap.dcMotor.get("Shooter");

        Wobbler = hardwareMap.dcMotor.get("Wobbler");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");

        /* DISTANCE SENSORS */
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        FrontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        /* MODE SWITCHES */
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


        /*IMU INIT */
        imu = hardwareMap.get( BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        initialVoltage = voltageSensor.getVoltage();

        //Initialization
        while (!(isStopRequested() || isStarted())) {


            //Update Neccessary Variables depending on Ring Configuration
            if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {

                encodersToDrop = 1410;
                angleToDrop = -35;
                diagonalDistance = 13;
                secondWobbleGoalDistance = 45;
                secondWobbleEncoderDistance = 600;
                position = 3;
                movement = UPLEFT;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {

                angleToDrop = -32;
                encodersToDrop = 690;
                diagonalDistance = 20;
                secondWobbleGoalDistance = 37;
                secondWobbleEncoderDistance = 400;
                position = 2;
                movement = DISTANCELEFT;
                angleForSecond = -342;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.A){

                encodersToDrop = 520;
                angleToDrop = -77;
                diagonalDistance = 15;
                secondWobbleGoalDistance = 52;
                secondWobbleEncoderDistance = 100;
                position = 1;
                movement = DISTANCELEFT;

            }


            telemetry.addData("Value", pipeline.getAnalysis());
            telemetry.addData("ringConfig", pipeline.configuration);
            telemetry.addData("parrallelEncoder", LeftForward.getCurrentPosition());
            telemetry.addData("getAngle", getAngle());
            telemetry.addData("Leftbackposition", LeftBack.getCurrentPosition());
            telemetry.addData("Right Wheel", LeftForward.getCurrentPosition());
            telemetry.addData("encodersToDrop", encodersToDrop);
            telemetry.addData("angleToDrop", angleToDrop);
            telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Time Elapsed", runtime);
            telemetry.addLine( ">>>INITIALIZATION COMPLETED");
            telemetry.update();

            sleep(50);
        }

        if (opModeIsActive() && !isStopRequested()) {

            strafe = new PIDController(0.016367*1.57, 0.00016367*1.5, 0);
            strafe.setSetpoint(0);
            strafe.setOutputRange(0, 0.75);
            strafe.setInputRange(-90, 90);
            strafe.enable();

            drive = new PIDController(0.016367*1.5, 0.00016367*5, 0.00016367*3);
            drive.setSetpoint(0);
            drive.setOutputRange(0, 0.75);
            drive.setInputRange(-90, 90);
            drive.enable();

            backDrive = new PIDController(0.016367*0.7, 0.00016367, 0);
            backDrive.setSetpoint(0);
            backDrive.setOutputRange(0, 0.75);
            backDrive.setInputRange(-90, 90);
            backDrive.enable();


            diagonal = new PIDController(0.016367*6, 0, 0);
            diagonal.setSetpoint(0);
            diagonal.setOutputRange(0, 0.75);
            diagonal.setInputRange(-90, 90);
            diagonal.enable();





            moveEncoders(Forward, 0.6, 1240, 0, 0);

            sleep(400);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            imuTurn(RTURN, 0.2, 3);

            sleep(200);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            powershot();

            sleep(200);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //C and B = -35
            //A = -86

            imuTurn(RTURN, 0.4, angleToDrop);

            sleep(50);


            //C = 1300
            //B = 450
            //A = 500

            moveEncoders(FORWARD_WITH_ARM, 0.6, encodersToDrop, angleToDrop, 2);

            openWobbleClamper();

            if(position == 3) {

                LeftForward.setPower(-0.7);
                RightForward.setPower(-0.7);
                LeftBack.setPower(-0.7);
                RightBack.setPower(-0.7);

                sleep(1200);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

                if (initialVoltage > 13.5) {
                    moveEncoders(RIGHT,0.58, 19, -90, 2);
                } else {
                    moveEncoders(RIGHT,0.58, 17, -90, 2);
                }

                sleep(100);

                moveEncoders(TURNED_FORWARD, 0.7, 150, -90,0);

                sleep(50);

                moveEncoders(FORWARDWITHFRONT, 0.25, 30, -90, 4);

                sleep(200);

                closeWobbleClamper();

                sleep(300);

                liftWobbleGoal(1400);

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                LeftForward.setPower(-0.7);
                RightForward.setPower(-0.7);
                LeftBack.setPower(-0.7);
                RightBack.setPower(-0.7);

                sleep(800);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                imuTurn(LTURN, 0.4,-46);

                sleep(50);

               moveEncoders(TURNED_FORWARD, 0.7, 2300, -30, 6);

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                imuTurn(RTURN, 0.3, -33);

                dropWobbleGoal();

                sleep(100);

                liftWobbleGoal(2000);

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                LeftForward.setPower(-0.7);
                RightForward.setPower(-0.7);
                LeftBack.setPower(-0.7);
                RightBack.setPower(-0.7);

                sleep(800);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

            } else if (position == 2) {
                sleep(200);

                LeftForward.setPower(-0.6);
                RightForward.setPower(-0.6);
                LeftBack.setPower(-0.6);
                RightBack.setPower(-0.6);

                sleep(300);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                imuTurn(RTURN, 0.5, -83);

                moveEncoders(BACKWARD, 0.5, 40, -90, .5);

                sleep(50);

                if (initialVoltage > 13.1) {
                    moveEncoders(RIGHT, 0.58, 18, -90, 5);
                } else {
                    moveEncoders(RIGHT, 0.64, 17, -90, 5);
                }
                sleep(50);

                moveEncoders(TURNED_FORWARD, 0.7, 120, -90, 5);

                sleep(10);

                moveEncoders(FORWARDWITHFRONT, 0.25, 32, -90, 5);

                sleep(600);

                closeWobbleClamper();

                sleep(100);

                liftWobbleGoal(1400);

                sleep(200);

                LeftForward.setPower(-0.6);
                RightForward.setPower(-0.6);
                LeftBack.setPower(-0.6);
                RightBack.setPower(-0.6);

                sleep(500);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

                sleep(50);

                moveEncoders(LEFT, 0.7, 2300, -90, 5);

                sleep(50);

                moveEncoders(TURNED_FORWARD, 0.5, 260, -90, 5);

                dropWobbleGoal();

                sleep(50);

                liftWobbleGoal(3000);

                LeftForward.setPower(-0.6);
                RightForward.setPower(-0.6);
                LeftBack.setPower(-0.6);
                RightBack.setPower(-0.6);

                sleep(500);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);

                moveEncoders(ENCODER_RIGHT, 0.6, 200, -90, 5);

                liftWobbleGoal(16367/2 - 900 - 3000);


            }
            else if (position == 1) {

                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                LeftBack.setPower(-0.4);
                LeftForward.setPower(-0.4);
                RightForward.setPower(-0.4);
                RightBack.setPower(-0.4);

                sleep(1100);

                LeftBack.setPower(0);
                LeftForward.setPower(0);
                RightForward.setPower(0);
                RightBack.setPower(0);

                sleep(100);

                if (initialVoltage > 13) {
                    moveEncoders(RIGHT, 0.58, 19, -90, 5);
                } else {
                    moveEncoders(RIGHT, 0.64, 19, -90, 5);
                }
                sleep(50);
                moveEncoders(FORWARDWITHFRONT, 0.25, 28, -90, 5);
                sleep(600);
                closeWobbleClamper();
                sleep(500);
                liftWobbleGoal(1400);

                moveEncoders(LEFT, 0.7, 1970, 0, 5);

                sleep(50);


                if (position == 1){
                    LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //moveEncoders(TURNED_FORWARD, 0.7, 50, -90);



                    LeftForward.setPower(0.7);
                    RightForward.setPower(0.7);
                    LeftBack.setPower(0.7);
                    RightBack.setPower(0.7);

                    sleep(200);

                    LeftForward.setPower(0);
                    RightForward.setPower(0);
                    LeftBack.setPower(0);
                    RightBack.setPower(0);

                    dropWobbleGoal();

                    sleep(50);


                }

                liftWobbleGoal(2000);

                sleep(50);

                LeftForward.setPower(-0.6);
                RightForward.setPower(-0.6);
                LeftBack.setPower(-0.6);
                RightBack.setPower(-0.6);

                sleep(300);

                LeftForward.setPower(0);
                RightForward.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);




            }

            //liftWobbleGoal(16367/2 - 900);

        }

    }

    public void moveEncoders(int Direction, double Power, int TargetPosition, int desiredAngle, double failSafeTime) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (failSafeTime > 0){
            runtime.reset();
        }

        if (Direction == Forward) {

            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (isStarted() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

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


        } else if (Direction == TURNED_FORWARD) {

            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

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


        }else if (Direction == FORWARD_WITH_ARM) {

            Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftBack.setTargetPosition(TargetPosition);
            Wobbler.setTargetPosition(-16367/2 + 1590);
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Wobbler.setPower(1);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition()) && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();
                if (Math.abs(Wobbler.getCurrentPosition()) >= Math.abs(Wobbler.getTargetPosition()))
                    Wobbler.setPower(0);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);

            while (!isStopRequested() && Math.abs(Wobbler.getCurrentPosition()) <= Math.abs(Wobbler.getTargetPosition())) {

                telemetry.addLine("wobble going");
            }

            Wobbler.setPower(0);




        }else if (Direction == TURNED_BACKWARD) {

            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();


            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == BACKWARD) {

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!isStopRequested() && BackDistance.getDistance(DistanceUnit.INCH) >= TargetPosition) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == LEFT) {
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double angle = getAngle();

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - angle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        }else if (Direction == TURNED_RIGHT){

            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);

        }else if (Direction == DISTANCELEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) > TargetPosition) {

                correction = strafe.performPID(getAngle() + 180);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == RIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > TargetPosition && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == ENCODER_RIGHT){

            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                LeftForward.setPower(Power);
                LeftBack.setPower(-Power);
                RightForward.setPower(-Power);
                RightBack.setPower(Power);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);

        }else if (Direction == LTURN) {

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
        else if (Direction == UPLEFT) {

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double angle = getAngle();

            while (opModeIsActive() && !isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) >= TargetPosition|| LeftBack.getCurrentPosition() < TargetPosition) {

                correction = diagonal.performPID(getAngle() - angle);

                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power - correction);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == FORWARDWITHFRONT) {
            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && !isStopRequested() && FrontDistance.getDistance(DistanceUnit.INCH) >= TargetPosition && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addData("frontdistance", FrontDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        }/*else if (Direction == FORWARDWITHBACK) {
            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double angle = getAngle();

            while (opModeIsActive() && !isStopRequested() && BackDistance.getDistance(DistanceUnit.INCH) <= TargetPosition || LeftBack.getCurrentPosition() < failsafe) {

                correction = backDrive.performPID(getAngle() - angle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        }
*/
        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

    }


    public void closeWobbleClamper() {
        WobbleClamper.setPosition(1);
    }

    public void openWobbleClamper () {
        WobbleClamper.setPosition(0.5);
    }


    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

    }


    private int inchesToTicksParallel(int TargetPosition) {
        return (int) Math.round((TargetPosition * TICKS_PER_REV_PARALLEL) / (WHEEL_RADIUS * 2 * Math.PI));
    }

    private int inchesToTicksPerpindicular(int TargetPosition) {
        return (int) Math.round((TargetPosition * TICKS_PER_REV_PERPINDICULAR) / (WHEEL_RADIUS * 2 * Math.PI));
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

    public static double encoderTicksToInchesParallel(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / 1460;
    }

    public void imuTurn(int direction, double power, double angle) {

        if (direction == LTURN){
            LeftForward.setPower(-power);
            LeftBack.setPower(-power);
            RightForward.setPower(power);
            RightBack.setPower(power);

            while (opModeIsActive() && getAngle() <= angle){
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

            while (opModeIsActive() && getAngle() >= angle){
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


    public void powershot(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i<3; i++) {


            if (i == 0)
                Shooter.setPower(-0.515);
            else if (i == 1)
                Shooter.setPower(-0.52);
            else
                Shooter.setPower(-0.505);

            sleep(1300);

            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(i==0)
                Conveyor.setTargetPosition(180);
            else if (i == 1)
                Conveyor.setTargetPosition(650);
            else if (i == 2)
                Conveyor.setTargetPosition(1000);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(-0.8);
            Conveyor.setPower(0.9);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

            }

            Intake.setPower(0);
            Conveyor.setPower(0);
            Shooter.setPower(0);

            sleep(400);

            if(i == 0){
                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                imuTurn(LTURN, 0.2, 5.5);

            } else if(i == 1){
                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                imuTurn(LTURN, 0.2, 10.75);
            }
            sleep(400);

        }


        Shooter.setPower(0);

    }


    public void dropWobbleGoal() {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Wobbler.setTargetPosition(-2000);

        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Wobbler.setPower(1);

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition())) {
            telemetry.addData("Dropping", "Right Now");
            telemetry.update();
        }

        Wobbler.setPower(0);
        openWobbleClamper();
    }

    public void liftWobbleGoal(int encoders) {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Wobbler.setTargetPosition(encoders);

        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Wobbler.setPower(1);

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition())) {
            telemetry.addData("Dropping", "Right Now");
            telemetry.update();
        }

        Wobbler.setPower(0);
        //openWobbleClamper();
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(330,90);

        static final int REGION_WIDTH = 95;
        static final int REGION_HEIGHT = 70;

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
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingDeterminationPipeline.RingConfiguration configuration = RingDeterminationPipeline.RingConfiguration.C;

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

            configuration = RingDeterminationPipeline.RingConfiguration.C; // Record our configuration

            if(avg1 > FOUR_RING_THRESHOLD){
                configuration = RingDeterminationPipeline.RingConfiguration.C;
            }else if (avg1 > ONE_RING_THRESHOLD){
                configuration = RingDeterminationPipeline.RingConfiguration.B;
            }else{
                configuration = RingDeterminationPipeline.RingConfiguration.A;
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
