package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BuildersTeleOp_AS", group = "")
public class BuildersTeleOp_AS extends LinearOpMode {

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

    //Align Variables
    public double inchesToVerticalAlignment = 45;
    public double inchesToHorizontalAlignment = 37;

 private double Scale (double Input) {
      double Output = Input * Math.abs(Input);
      return Output;
 }

  @Override
  public void runOpMode() throws InterruptedException {

    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");

    Intake = hardwareMap.dcMotor.get("Intake");
    Conveyor = hardwareMap.dcMotor.get("Conveyor");
    Shooter = hardwareMap.dcMotor.get("Shooter");

    Wobbler = hardwareMap.dcMotor.get("Wobbler");

      RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
      BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
      FrontDistance = hardwareMap.get(DistanceSensor.class, "FrontDistance");
      LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");


    WobbleClamper = hardwareMap.servo.get("WobbleClamper");
    /*
    Ringer = hardwareMap.dcMotor.get("Ringer");
    RingClamper = hardwareMap.servo.get("RingClamper");
    
    */

    RightBack.setDirection(DcMotor.Direction.REVERSE);
    LeftForward.setDirection(DcMotor.Direction.REVERSE);



    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Wobbler.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    WobbleClamper.setPosition(0.3);
    
    RightForward.setPower(0);
    RightBack.setPower(0);
    LeftForward.setPower(0);
    LeftBack.setPower(0);
    
   // WobbleClamper.setPosition(0);
    telemetry.addData(">", "INIT DONE");
// RESET TIME
    runtime.reset();

    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
        while (opModeIsActive()) {
       if(gamepad1.right_trigger > 0.01){
         // Strafing to the Right
        LeftForward.setPower(-Multiplier * Scale(gamepad1.right_trigger));
        LeftBack.setPower(Multiplier * Scale(gamepad1.right_trigger));
        RightForward.setPower(Multiplier * Scale(gamepad1.right_trigger));
        RightBack.setPower(-Multiplier * Scale(gamepad1.right_trigger));
       } else if(gamepad1.left_trigger > 0.01){
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
       } else if (gamepad1.right_bumper){
           RightBack.setPower(0.7);
           RightForward.setPower(-0.7);
           LeftForward.setPower(0.7);
           LeftBack.setPower(-0.7);
       } else if (gamepad1.left_bumper) {
           RightBack.setPower(-0.7);
           RightForward.setPower(0.7);
           LeftForward.setPower(-0.7);
           LeftBack.setPower(0.7);
       } else if (gamepad1.y) {
            //align(0.7);
       } else if (gamepad1.dpad_left) {
           RightForward.setPower(-0.7);
           LeftBack.setPower(-0.7);
       } else if (gamepad1.dpad_right) {
           LeftForward.setPower(-0.7);
           RightBack.setPower(-0.7);
       }
       else {
        RightBack.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        RightForward.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        /*Ringer.setPower(gamepad2.left_stick_y);
        Wobbler.setPower(gamepad2.right_stick_y);*/
      }



      if (gamepad2.right_stick_y > 0.01 || gamepad2.right_stick_y < -0.01) {
          Intake.setPower(0.8 * Scale(gamepad2.right_stick_y));
          Conveyor.setPower(-0.7 * Scale(gamepad2.right_stick_y));
      } else {
          Intake.setPower(0);
          Conveyor.setPower(0);
      }

      if (gamepad2.left_stick_y > 0.01 || gamepad2.left_stick_y < -0.01) {
         Wobbler.setPower(-gamepad2.left_stick_y);
      } else {
        Wobbler.setPower(0);
      }

      if (gamepad2.dpad_up) {
          Shooter.setPower(-0.83);
      } else if (gamepad2.dpad_left) {
          Shooter.setPower(-0.79);
      } else {
          Shooter.setPower(0);
      }

      if (gamepad2.left_bumper) { //OUT
        WobbleClamper.setPosition(0.3);
      } else if (gamepad2.right_bumper){ //IN
          WobbleClamper.setPosition(1);
      }

      telemetry.addData("Conveyor + Intake", Conveyor.getPower());
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

      
      
      


        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
      }
    }



  }

    public void moveDistance(int Direction, double Power, double distance){

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


         if (Direction == FORWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) < distance) {


                LeftForward.setPower(Power);
                LeftBack.setPower(Power);
                RightForward.setPower(Power);
                RightBack.setPower(Power);

                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }else if (Direction == BACKWARD) {
            while (opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) > distance) {


                LeftForward.setPower(-Power);
                LeftBack.setPower(-Power);
                RightForward.setPower(-Power);
                RightBack.setPower(-Power);

                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == RIGHT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > distance) {


                LeftForward.setPower(Power);
                LeftBack.setPower(-Power);
                RightForward.setPower(-Power);
                RightBack.setPower(Power);

                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (Direction == LEFT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) < distance) {


                LeftForward.setPower(-Power);
                LeftBack.setPower(Power);
                RightForward.setPower(Power);
                RightBack.setPower(-Power);

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


    public void align(double power) {


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

    public void dropWobbleGoal() {
        Wobbler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Wobbler.setTargetPosition(-8000);

        Wobbler.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Wobbler.setPower(1);

        while (opModeIsActive() && Math.abs(Wobbler.getCurrentPosition()) < Math.abs(Wobbler.getTargetPosition())) {
            telemetry.addData("Dropping", "Right Now");
            telemetry.update();
        }

        Wobbler.setPower(0);
        WobbleClamper.setPosition(0.3);
    }

}




