package org.firstinspires.ftc.teamcode.autos;
import android.drm.DrmStore;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.TurnTest;
import org.firstinspires.ftc.teamcode.openCV.colorReact;
import org.firstinspires.ftc.teamcode.openCV.wobbleDetection;
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
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PERPINDICULAR;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.TICKS_PER_REV_PARALLEL;
import static org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer.WHEEL_RADIUS;

import static org.firstinspires.ftc.teamcode.openCV.colorReact.threshold;



@Autonomous(name = "DELAYEDRedPowerShotWobble", group = "autos")
public class DELAYEDRedPowerShotWobble extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer, Intake, Conveyor;
    public DcMotorEx Shooter;
    public Servo WobbleClamper, RingClamper, Hopper;
    public DistanceSensor RightDistance, FrontDistance, LeftDistance;

    public OpenCvCamera webcam;
    public OpenCvCamera backcam;
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
    PIDController leftStrafe;
    PIDController cDrive;
    PIDController shooter;

    public TouchSensor WobbleTouch;

    public Encoder frontEncoder;

    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;
    int Forward = 7;
    int UPLEFT = 8;
    int UPRIGHT = 9;
    int FORWARD_WITH_ARM = 14;
    int cForward = 15;
    int RTURN_WITHSHOOT = 17;
    int DISTANCELEFT = 16;

    public static int xPos1 = 20;
    public static int yPos1 = 347;

    public static int width1 = 95;
    public static int height1 = 77;

    boolean checkFrontDist = false;

    public static int initialAmt;
    public static int currentPosition;

    public static int orangeThreshold = 170;
    public static int horizon = 300;
    public static int ringHorizon = 300;
    public static int verticalHorion = 153;
    public static int verticalHorion2 = 420;
    public static boolean highGoal = true;

    private VoltageSensor voltageSensor;
    private double initialVoltage;

    private RevBlinkinLedDriver blinkblinkboy;

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

    public static double armSpeed;

    public static final int frameHeight= 640;
    public static final int frameWidth = 480;


    private int value;
    private int wobbleValue;

    private static boolean shoot = false;

    @Override
    public void runOpMode() throws InterruptedException {


        /* DRIVE MOTORS */
        LeftForward = hardwareMap.get(DcMotor.class, "LeftForward");
        RightForward = hardwareMap.get(DcMotor.class, "RightForward");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightForward"));

        /* SUBSYSTEM */
        Intake = hardwareMap.dcMotor.get("Intake");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        Wobbler = hardwareMap.dcMotor.get("Wobbler");

        blinkblinkboy = hardwareMap.get(RevBlinkinLedDriver.class, "blinkblinkboy");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");
        Hopper = hardwareMap.servo.get("Hopper");

        WobbleTouch = hardwareMap.get(TouchSensor.class, "WobbleTouch");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        /* MODE SWITCHES */
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftForward.setDirection(DcMotor.Direction.REVERSE);

        Intake.setDirection(DcMotor.Direction.REVERSE);


        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(4, 2, 1, 1));
        Wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeWobbleClamper();


        /*IMU INIT */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally


        backcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

        backcam.openCameraDevice();
        webcam.openCameraDevice();

        pipeline = new RingDeterminationPipeline();

        webcam.setPipeline(pipeline);

        backcam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPSIDE_DOWN);
        webcam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        initialVoltage = voltageSensor.getVoltage();


        //Initialization
        while (!(isStopRequested() || isStarted())) {


            //Update Neccessary Variables depending on Ring Configuration
            /*if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.C) {

                encodersToDrop = 1845;
            angleToDrop = 30;
            armSpeed = -0.45;
            diagonalDistance = 13;
            secondWobbleGoalDistance = 45;
            secondWobbleEncoderDistance = 600;
            position = 3;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.B) {

            angleToDrop = -20;
            encodersToDrop = 1050;
            armSpeed = -0.7;
            diagonalDistance = 20;
            secondWobbleGoalDistance = 37;
            secondWobbleEncoderDistance = 400;
            position = 2;
            angleForSecond = -342;

            } else if (pipeline.configuration == RingDeterminationPipeline.RingConfiguration.A){

               encodersToDrop = 970;
            angleToDrop = 69;
            armSpeed = -0.7;
            diagonalDistance = 15;
            secondWobbleGoalDistance = 52;
            secondWobbleEncoderDistance = 100;
            position = 1;

            }*/

            angleToDrop = -20;
            encodersToDrop = 1050;
            armSpeed = -0.6;
            diagonalDistance = 20;
            secondWobbleGoalDistance = 37;
            secondWobbleEncoderDistance = 400;
            position = 2;
            angleForSecond = -342;


            highGoal = true;
            verticalHorion = 250;

            RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("parrallelEncoder", RightForward.getCurrentPosition());
            telemetry.addData("getAngle", getAngle());
            telemetry.addData("Leftbackposition", LeftBack.getCurrentPosition());
            telemetry.addData("Right Wheel", LeftForward.getCurrentPosition());
            telemetry.addData("encodersToDrop", encodersToDrop);
            telemetry.addData("angleToDrop", angleToDrop);
            telemetry.addData("Time Elapsed", runtime);
            telemetry.addLine(">>>INITIALIZATION COMPLETED");
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {

            strafe = new PIDController(0.0016367 * 1.5, 0.00016367, 0.000016367);
            strafe.setSetpoint(0);
            strafe.setOutputRange(0, 0.75);
            strafe.setInputRange(-90, 90);
            strafe.enable();

            drive = new PIDController(0.016367 * 0.7, 0.00016367 * 5, 0.000016367 * 6);
            drive.setSetpoint(0);
            drive.setOutputRange(0, 0.5);
            drive.setInputRange(-90, 90);
            drive.enable();

            backDrive = new PIDController(0.016367 * 0.7, 0.00016367, 0);
            backDrive.setSetpoint(0);
            backDrive.setOutputRange(0, 0.75);
            backDrive.setInputRange(-90, 90);
            backDrive.enable();

            diagonal = new PIDController(0.016367 * 6, 0, 0);
            diagonal.setSetpoint(0);
            diagonal.setOutputRange(0, 0.75);
            diagonal.setInputRange(-90, 90);
            diagonal.enable();

            leftStrafe = new PIDController(0.0016367 * 2.5, 0.00016367, 0.000016367);
            leftStrafe.setSetpoint(0);
            leftStrafe.setOutputRange(0, 0.75);
            leftStrafe.setInputRange(-90, 90);
            leftStrafe.enable();

            cDrive = new PIDController(0, 0, 0);
            cDrive.setSetpoint(0);
            cDrive.setOutputRange(0, 0.75);
            cDrive.setInputRange(-90, 90);
            cDrive.enable();

            shooter = new PIDController(0.016367, 0, 0);
            shooter.setSetpoint(0.53);
            shooter.setOutputRange(-0.1, 0.1);
            shooter.setInputRange(0, 1);
            shooter.enable();

            RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(10*1000);

//GO TO POWERSHOTS
            Shooter.setPower(.52);

            moveEncoders(Forward, 0.7, 1700, 2.5);

            sleep(1000);

            //imuTurn(RTURN, 0.23, -2.5);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            imuTurn(RTURN, 0.28, 1);

            powershot();

            highGoal = false;

            sleep(200);

            //C and B = -35
            //A = -86
//GO TO TARGET ZONE
            imuTurn(RTURN, 0.4, angleToDrop);

            sleep(50);

            moveEncoders(FORWARD_WITH_ARM, 0.6, encodersToDrop, angleToDrop);

            openWobbleClamper();

            //liftWobbleGoal(16367/2 - 900 - 800);

        }

    }

    public void moveDistance(int Direction, double Power, double Distance, double desiredAngle, double failSafeTime) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetStartTime();

        if (Direction == LEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance) {

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


            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && time < failSafeTime)) {

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


        } else if (Direction == UPLEFT) {

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double angle = getAngle();

            while (opModeIsActive() && !isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) >= Distance && (failSafeTime > 0 && time < failSafeTime)) {

                correction = diagonal.performPID(getAngle() - angle);

                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power - correction);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == FORWARD) {
            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && !isStopRequested() && FrontDistance.getDistance(DistanceUnit.INCH) >= Distance &&  (failSafeTime > 0 && time < failSafeTime)) {

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
        }
        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

    }

    public void moveEncoders(int Direction, double Power, int TargetPosition, double desiredAngle) {
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetStartTime();

        if (Direction == Forward){
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle() - desiredAngle);

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
        }else if(Direction == cForward){
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = cDrive.performPID(getAngle() - desiredAngle);

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
        } else if (Direction == BACKWARD){
            LeftBack.setTargetPosition(-TargetPosition);

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
        }else if (Direction == LEFT){
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = leftStrafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == RIGHT){
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (opModeIsActive() && !isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);
        } else if (Direction == FORWARD_WITH_ARM){

            Wobbler.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Wobbler.setPower(armSpeed);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition()) ) {

                correction = backDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);


                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.update();
                if (WobbleTouch.isPressed())
                    Wobbler.setPower(0);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);

            while (!isStopRequested() && !WobbleTouch.isPressed()) {

                telemetry.addLine("wobble going");
            }

            Wobbler.setPower(0);

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
    }

    public void righStrafeWithCenterWheel(int inches){

        int zeroPoint = frontEncoder.getCurrentPosition();

        double encodersToRotate = Math.round(inches * 8192 / (.69 * 2 * Math.PI));

        while(Math.abs(frontEncoder.getCurrentPosition() - zeroPoint) <= Math.abs(encodersToRotate)){

            RightForward.setPower(-0.23);
            RightBack.setPower(-0.23);
            LeftForward.setPower(0.23);
            LeftBack.setPower(0.23);

            telemetry.addData("encoderCount", frontEncoder.getCurrentPosition() - initialAmt);
            telemetry.update();

        }

        RightForward.setPower(0);
        RightBack.setPower(0);
        LeftForward.setPower(0);
        LeftBack.setPower(0);
    }


    public void closeWobbleClamper() {
        WobbleClamper.setPosition(1);
    }

    public void openWobbleClamper () {
        WobbleClamper.setPosition(0.353);
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

    public void adjustmentTurn(double power, int target) {

        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightForward.setTargetPosition(target);

        RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftForward.setPower(power);
        LeftBack.setPower(power);
        RightForward.setPower(-power);
        RightBack.setPower(-power);

        while (opModeIsActive() && Math.abs(RightForward.getCurrentPosition()) <= Math.abs(RightForward.getTargetPosition())){

            telemetry.addData("currentEncoderPosition", RightForward.getCurrentPosition());
            telemetry.addLine("Turning :)");
            telemetry.update();
        }

        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);

    }

    public void imuTurn(int direction, double power, double angle) {

        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        } else if(direction == RTURN_WITHSHOOT){
            LeftForward.setPower(power);
            LeftBack.setPower(power);
            RightForward.setPower(-power);
            RightBack.setPower(-power);
            Shooter.setPower(0.53);
            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Conveyor.setTargetPosition(800);

            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Conveyor.setPower(-0.9);


            while (opModeIsActive() && getAngle() >= angle){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();

                if(Conveyor.getCurrentPosition() > Conveyor.getTargetPosition()){
                    Conveyor.setPower(0);
                }
            }

            LeftForward.setPower(0);
            LeftBack.setPower(0);
            RightForward.setPower(0);
            RightBack.setPower(0);
            Conveyor.setPower(0);

            while (!isStopRequested() && Conveyor.getCurrentPosition() < Conveyor.getTargetPosition()) {

                telemetry.addLine("conveyor going");
            }

            Conveyor.setPower(0);
        }
    }


    public void powershot(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i<3; i++) {


            if(i==0)
                Shooter.setPower(0.55);
            else if(i==1)
                Shooter.setPower(0.521);
            else if(i==2)
                Shooter.setPower(0.494);


            /*if (i == 0)
                Shooter.setPower(0.515);
            else if (i == 1)
                Shooter.setPower(0.52);
            else
                Shooter.setPower(0.505);
*/
            if(i==0) {
                closeWobbleClamper();
                sleep(200);
            }
            else if (i==1 || i==2) {
                closeWobbleClamper();
                sleep(100);
            }
            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(i==0)
                Conveyor.setTargetPosition(180);
            else if (i == 1)
                Conveyor.setTargetPosition(650);
            else if (i == 2)
                Conveyor.setTargetPosition(1700);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(-0.8);
            Conveyor.setPower(0.9);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

            }

            Intake.setPower(0);
            Conveyor.setPower(0);
            //Shooter.setPower(0);

            sleep(80);

            if(i == 0){

                imuTurn(LTURN, 0.25, 4);



            } else if(i == 1){

                imuTurn(LTURN, 0.25, 8);

            }
            sleep(60);

        }


        Shooter.setPower(0);

    }


    public void shootC(){

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (int i = 0; i<3; i++) {
            sleep(500);

            Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if(i==0)
                Conveyor.setTargetPosition(180);
            else if (i == 1)
                Conveyor.setTargetPosition(650);
            else if (i == 2)
                Conveyor.setTargetPosition(1500);


            Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Intake.setPower(-0.8);
            Conveyor.setPower(0.9);

            while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

            }

            Intake.setPower(0);
            Conveyor.setPower(0);
            //Shooter.setPower(0);

            sleep(80);
        }


        Shooter.setPower(0);

    }

    public void shoot() {

        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Conveyor.setTargetPosition(1400);

        Conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Intake.setPower(-0.8);
        Conveyor.setPower(0.9);

        while (opModeIsActive() && !isStopRequested() && Math.abs(Conveyor.getCurrentPosition()) <= Math.abs(Conveyor.getTargetPosition())) {

        }

        Intake.setPower(0);
        Conveyor.setPower(0);
        //Shooter.setPower(0);

        sleep(80);

        Shooter.setPower(0);

    }


    public void dropWobbleGoal() {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wobbler.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Wobbler.setPower(-1);

        while (!isStopRequested() && opModeIsActive() && !WobbleTouch.isPressed()) {
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

        Wobbler.setPower(0.7);

        runtime.reset();

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition()) && runtime.seconds() < 1.3) {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(xPos1,yPos1);

        static final int REGION1_WIDTH = width1;
        static final int REGION1_HEIGHT = height1;
        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 133;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingConfiguration configuration = RingConfiguration.C;

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

            inputToCb(input);

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }


    }
}
