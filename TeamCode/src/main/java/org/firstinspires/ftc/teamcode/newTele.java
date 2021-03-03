package org.firstinspires.ftc.teamcode;
// import Rahul's Genius & IQ
// import GOAT || AMAN
// import Da One And Only Muthu
//import com.qualcomm.robotcore.brain.Moni;
import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp(name = "newTele", group = "")
public class newTele extends LinearOpMode {

    private DcMotor RightForward, RightBack, LeftForward, LeftBack, Intake, Shooter, Conveyor, Wobbler;
    private Servo WobbleClamper, RingClamper;
    private DistanceSensor BackDistance, RightDistance, FrontDistance, LeftDistance;
    private ElapsedTime runtime = new ElapsedTime();
    private double Multiplier = 0.7;
    public double contPower;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;
    int FORWARDWITHFRONT = 6;
    int Forward = 7;
    int RIGHTWITHLEFT = 8;
    int i = 1;
    static double conveyorPower;

    boolean endgame = false;

    public static double ringDistance;

    public RevBlinkinLedDriver blinkblinkboy;
    public ColorSensor color;
    public TouchSensor WobbleTouch;

    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    double globalAngle, correction, rotation;
    PIDController strafe;


    private VoltageSensor voltageSensor;

    //Align Variables
    public double inchesToVerticalAlignment = 45;
    public double inchesToHorizontalAlignment = 37;

    private double Scale(double Input) {
        double Output = Input * Math.abs(Input);
        return Output;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");



        WobbleTouch = hardwareMap.get(TouchSensor.class, "WobbleTouch");

        //blinkblinkboy = hardwareMap.get(RevBlinkinLedDriver.class, "blinkblinkboy");

        // color = hardwareMap.get(ColorSensor.class, "color");

        Intake = hardwareMap.dcMotor.get("Intake");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        Shooter = hardwareMap.dcMotor.get("Shooter");

        Wobbler = hardwareMap.dcMotor.get("Wobbler");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        //BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        FrontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");
    /*
    Ringer = hardwareMap.dcMotor.get("Ringer");
    RingClamper = hardwareMap.servo.get("RingClamper");

    */

        imu = hardwareMap.get( BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        RightBack.setDirection(DcMotor.Direction.REVERSE);
        RightForward.setDirection(DcMotor.Direction.REVERSE);

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  WobbleClamper.setPosition(0.7);

        RightForward.setPower(0);
        RightBack.setPower(0);
        LeftForward.setPower(0);
        LeftBack.setPower(0);



        WobbleClamper.setPosition(0);
        telemetry.addData(">", "INIT DONE");
// RESET TIME
        runtime.reset();

        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {

            strafe = new PIDController(0.0016367, 0.00016367, 0.000016367);
            strafe.setSetpoint(0);
            strafe.setOutputRange(0, 0.5);
            strafe.setInputRange(-90, 90);
            strafe.enable();

            runtime.reset();

            while (opModeIsActive()) {

                //ringDistance =  ((DistanceSensor)color).getDistance(DistanceUnit.CM);

               /* if(time > 5){
                    endgame = true;
                } else if(gamepad1.dpad_right){
                    resetStartTime();
                }

*/
                if(gamepad1.b){
                    resetStartTime();
                }

                if (gamepad1.right_trigger > 0.01) {
                    // Strafing to the Right
                    LeftForward.setPower(-Multiplier * Scale(gamepad1.right_trigger));
                    LeftBack.setPower(Multiplier * Scale(gamepad1.right_trigger));
                    RightForward.setPower(Multiplier * Scale(gamepad1.right_trigger));
                    RightBack.setPower(-Multiplier * Scale(gamepad1.right_trigger));
                } else if (gamepad1.left_trigger > 0.01) {
                    // Strafing to the Left
                    LeftForward.setPower(Multiplier * Scale(gamepad1.left_trigger));
                    LeftBack.setPower(-Multiplier * Scale(gamepad1.left_trigger));
                    RightForward.setPower(-Multiplier * Scale(gamepad1.left_trigger));
                    RightBack.setPower(Multiplier * Scale(gamepad1.left_trigger));
                } //DcMotor Wobble Goal
                else if (gamepad1.dpad_up) { //Going forward slowly
                    RightBack.setPower(-0.3);
                    RightForward.setPower(-0.3);
                    LeftForward.setPower(-0.3);
                    LeftBack.setPower(-0.3);
        /*RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setTargetPosition(500);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setPower(0.75);*/

                } else if (gamepad1.dpad_down) { //Going backward slowly
                    RightBack.setPower(0.3);
                    RightForward.setPower(0.3);
                    LeftForward.setPower(0.3);
                    LeftBack.setPower(0.3);

        /*RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setTargetPosition(-500);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setPower(0.75);*/
                } else if (gamepad1.right_bumper) {

                    resetAngle();
                    imuTurn(RTURN, 0.3, 7);

                } else if (gamepad1.left_bumper) {

                }
                else if (gamepad1.y) {
                    //align(0.7);
                } else if (gamepad1.dpad_left) {
                    RightForward.setPower(-0.7);
                    LeftBack.setPower(-0.7);
                } else if (gamepad1.dpad_right) {
                    LeftForward.setPower(-0.7);
                    RightBack.setPower(-0.7);
                } else {
                    RightBack.setPower(Multiplier * Scale(gamepad1.right_stick_y));
                    RightForward.setPower(Multiplier * Scale(gamepad1.right_stick_y));
                    LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
                    LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));

        /*Ringer.setPower(gamepad2.left_stick_y);
        Wobbler.setPower(gamepad2.right_stick_y);*/
                }


                if (gamepad2.right_stick_y > 0.01 || gamepad2.right_stick_y < -0.01) {
                    //Intake.setPower(0.8 * Scale(gamepad2.right_stick_y));
                    Intake.setPower(1 * Scale(gamepad2.right_stick_y));

                    Conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Conveyor.setPower(-0.9 * Scale(gamepad2.right_stick_y));
                } else if (gamepad2.x) {
                    Conveyor.setPower(0.6);
                } else {
                    Intake.setPower(0);
                    Conveyor.setPower(0);
                }

                if (gamepad2.a) {
                    Conveyor.setPower(-0.4);
                } else if (gamepad1.back) {

                    //POWERSHOT SHOOTING

                    if (LeftDistance.getDistance(DistanceUnit.INCH) > 24) {
                        moveDistance(LEFT, 0.5, 24);
                    } else {
                        moveDistance(RIGHT, 0.5, 19);
                    }

                    sleep(100);

                    if (voltageSensor.getVoltage() > 13.1) {
                        Shooter.setPower(0.92);
                    } else {
                        Shooter.setPower(.96);
                    }

                    sleep(1300);

                    for (int i = 0; i<3; i++){
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

                        sleep(10);

                        resetAngle();
                        imuTurn(LTURN, 0.3, -7);

                    }


                }

                if (gamepad2.left_stick_y > 0.01 || gamepad2.left_stick_y < -0.01) {
                    Wobbler.setPower(-gamepad2.left_stick_y);
                } else {
                    Wobbler.setPower(0);
                }

                if (gamepad2.dpad_up) {
                    Shooter.setPower(0.54);
                } else if (gamepad2.dpad_down) {
                    Shooter.setPower(0.5);
                } else {
                    Shooter.setPower(0);
                }

                if (gamepad2.left_bumper) { //OUT
                    WobbleClamper.setPosition(0.7);
                } else if (gamepad2.right_bumper) { //IN
                    WobbleClamper.setPosition(1);
                }

                telemetry.addData("Conveyor + Intake", Conveyor.getPower());
                telemetry.addData("Conveyor Encoders", Conveyor.getCurrentPosition());
                telemetry.addData("Shooter Encoders", Shooter.getCurrentPosition());
                telemetry.addData("Shooter", Shooter.getPower());


      /*
      if (gamepad2.dpad_left) {
        Wobbler.setPower(1);

      } else if (gamepad2.dpad_right){
        Wobbler.setPower(-1);
      } else {
        Wobbler.setPower(0);
      }
*/
/*
                if (ringDistance < 3){

                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

                } else if(80 > time && time > 60){
                    // 30 seconds before endgame
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

                } else if(110 > time && time > 80){
                    //10 seconds before endgame
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                } else if(115 > time && time > 110){
                    //20 seconds before end
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);

                } else if (time < 120 && time > 115){
                    //5 seconds before end
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

                } else if(time > 120){
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                }else {
                    blinkblinkboy.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

                }
*/


                telemetry.addData("time", time);
                telemetry.addData("WobbleTouch", WobbleTouch.getValue());
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                //telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("FrontDistace", FrontDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Time", time);
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());


                telemetry.addData("Voltage", voltageSensor.getVoltage());
                // telemetry.addData("ringDistance", ((DistanceSensor)color).getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }


    }


    /*  public void align(double power) {


          LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


          LeftForward.setPower(0);
          RightForward.setPower(0);
          LeftBack.setPower(0);
          RightBack.setPower(0);

          //Move to Launch Line Vertically
          moveDistance(FORWARD, 0.7, inchesToVerticalAlignment);

          sleep(700);

          //Align Horizontally for Shooting

          if (RightDistance.getDistance(DistanceUnit.INCH) < inchesToHorizontalAlignment) {
              moveDistance(LEFT, 0.7, inchesToHorizontalAlignment);
          } else if (RightDistance.getDistance(DistanceUnit.INCH) > inchesToHorizontalAlignment) {
              moveDistance(RIGHT, 0.7, inchesToHorizontalAlignment + 2);
          }

          LeftForward.setPower(0);
          RightForward.setPower(0);
          LeftBack.setPower(0);
          RightBack.setPower(0);
      }
  */
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

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

    }
    public void imuTurn(int direction, double power, double angle) {

        if (direction == LTURN){
            LeftForward.setPower(-power);
            LeftBack.setPower(-power);
            RightForward.setPower(power);
            RightBack.setPower(power);

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
        } else if (direction == RTURN){
            LeftForward.setPower(power);
            LeftBack.setPower(power);
            RightForward.setPower(-power);
            RightBack.setPower(-power);

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
        }//aarav was here.
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
                resetAngle();
                imuTurn(LTURN, 0.3, -6.4);

            } else if(i == 1){
                LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                resetAngle();
                imuTurn(LTURN, 0.3, -6.4);
            }
            sleep(400);

        }


        Shooter.setPower(0);

    }

    public void moveDistance(int Direction, double Power, double Distance) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetStartTime();

        if (Direction == LEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double angle = getAngle();

            while (!isStopRequested() && opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance) {

                correction = strafe.performPID(getAngle() - angle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

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

            double angle = getAngle();

            while (!isStopRequested() && opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) < Distance) {

                correction = strafe.performPID(getAngle() - angle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

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


        }

    }

}


