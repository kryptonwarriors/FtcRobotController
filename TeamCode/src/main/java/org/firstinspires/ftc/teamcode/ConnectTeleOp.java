package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Disabled

@TeleOp(name = "ConnectTheTeleOp", group = "")
public class ConnectTeleOp extends LinearOpMode {

  private DcMotor RightForward, RightBack, LeftForward, LeftBack;
  private DcMotor LeftCascade, RightCascade;
  private DcMotor LinearActuator;
  private Servo  RightClamp, LeftClamp, RightTurner, RightClamper, LeftTurner, LeftClamper, BackClamper;
  private ElapsedTime runtime = new ElapsedTime();
  private double Multiplier = 0.7;
  private double StrafeMultiplier = -0.8;

 private double Scale (double Input) {
      double Output = Input * Math.abs(Input);
      return Output;
 }

  @Override
  public void runOpMode() {

    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    
    RightClamp = hardwareMap.servo.get("RightClamp");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    
    RightClamper = hardwareMap.servo.get("RightClamper");
    RightTurner = hardwareMap.servo.get("RightTurner");
    LeftClamper = hardwareMap.servo.get("LeftClamper");
    LeftTurner = hardwareMap.servo.get("LeftTurner");

    RightCascade = hardwareMap.dcMotor.get("RightCascade");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");


    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    RightForward.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    
    
    RightForward.setPower(0);
    RightBack.setPower(0);
    LeftForward.setPower(0);
    LeftBack.setPower(0);
    telemetry.addData(">", "INIT DONE");
// RESET TIME
    runtime.reset();

    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
       if(Math.abs(gamepad1.right_trigger) > 0.01){
         // Strafing to the Right
        LeftForward.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
        LeftBack.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightForward.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightBack.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
       } else if(Math.abs(gamepad1.left_trigger) > 0.01){
        // Strafing to the Left
        LeftForward.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
        LeftBack.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightForward.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightBack.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
      } else if (gamepad1.y) {
        RightBack.setPower(-0.4);
        RightForward.setPower(-0.4);
        LeftForward.setPower(-0.4);
        LeftBack.setPower(-0.4);
      } else if (gamepad1.a) {
        RightBack.setPower(0.4);
        RightForward.setPower(0.4);
        LeftForward.setPower(0.4);
        LeftBack.setPower(0.4);
      } else if (gamepad1.right_bumper) {
        RightBack.setPower(-0.8);
        RightForward.setPower(-0.8);
        LeftForward.setPower(0.8);
        LeftBack.setPower(0.8);
      } else if (gamepad1.left_bumper) {
        RightBack.setPower(0.8);
        RightForward.setPower(0.8);
        LeftForward.setPower(-0.8);
        LeftBack.setPower(-0.8);
      } else if (gamepad1.x == true) {

          RightClamp.setPosition(0.1);
          LeftClamp.setPosition(0.9);
          // Clamp in & CLOSE
        }
        else if (gamepad1.b == true) {

          RightClamp.setPosition(0.45);
          LeftClamp.setPosition(0.6);
          // Clamp out & OPEN
        }else if(gamepad1.dpad_down){
         RightCascade.setPower(-0.6);
         LeftCascade.setPower(0.6);
       }else if(gamepad1.dpad_up){
         RightCascade.setPower(0.6);
         LeftCascade.setPower(-0.6);
       }else {
        RightCascade.setPower(0.1);
        LeftCascade.setPower(-0.1);
        RightBack.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        RightForward.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LinearActuator.setPower(gamepad2.right_stick_y * -1);
      }

     /* if (gamepad2.x == true) {
          LeftFoundation.setPosition(0);
          RightFoundation.setPosition(0);
          // Down
        }
        if (gamepad2.y == true) {
          LeftFoundation.setPosition(0.6);
          RightFoundation.setPosition(0.91);
          // Up
          
        }*/
        if (gamepad2.left_bumper == true) {
          //Grab Stone
          RightTurner.setPosition(0.4);
          //LeftTurner.setPosition(1);
        }
        if (gamepad2.right_bumper == true) {
          //Up
          RightTurner.setPosition(1);
          //LeftTurner.setPosition(0);
        }
        if (gamepad2.dpad_up) {
          RightClamper.setPosition(0);
        } if (gamepad2.dpad_down) {
          RightClamper.setPosition(1);
  
        }
        /*if (gamepad2.left_stick_y == 0) {
          RightCascade.setPower(0.2);
          LeftCascade.setPower(-0.2);
        }
        */

        /*
        RightCascade.setPower(gamepad2.left_stick_y * -0.6);
        LeftCascade.setPower(gamepad2.left_stick_y * 0.6);
        */

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("LeftCascade", LeftCascade.getPower());
        telemetry.addData("RightCascade", RightCascade.getPower());
        telemetry.update();
      }
    }
  }
}

