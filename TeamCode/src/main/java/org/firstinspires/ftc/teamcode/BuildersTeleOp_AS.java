package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "BuildersTeleOp_AS", group = "")
public class BuildersTeleOp_AS extends LinearOpMode {

  private DcMotor RightForward, RightBack, LeftForward, LeftBack, Wobbler, Ringer;
  private Servo WobbleClamper, RingClamper;
  private ElapsedTime runtime = new ElapsedTime();
  private double Multiplier = 0.7;
  public double contPower;

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
    
    Wobbler = hardwareMap.dcMotor.get("Wobbler");

    WobbleClamper = hardwareMap.servo.get("WobbleClamper");
    
    Ringer = hardwareMap.dcMotor.get("Ringer");
    RingClamper = hardwareMap.servo.get("RingClamper");
    
    
    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    
    
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
      while (opModeIsActive()) {
       if(gamepad1.right_trigger > 0.01){
         // Strafing to the Right
        LeftForward.setPower(-Multiplier * Scale(gamepad1.right_trigger));
        LeftBack.setPower(Multiplier * Scale(gamepad1.right_trigger));
        RightForward.setPower(Multiplier * -Scale(gamepad1.right_trigger));
        RightBack.setPower(Multiplier * Scale(gamepad1.right_trigger));
       } else if(gamepad1.left_trigger > 0.01){
        // Strafing to the Left
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_trigger));
        LeftBack.setPower(-Multiplier * Scale(gamepad1.left_trigger));
        RightForward.setPower(Multiplier * Scale(gamepad1.left_trigger));
        RightBack.setPower(-Multiplier * Scale(gamepad1.left_trigger));
      } else {
        RightBack.setPower(-Multiplier * Scale(gamepad1.right_stick_y));
        RightForward.setPower(-Multiplier * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        Ringer.setPower(gamepad2.left_stick_y);
        Wobbler.setPower(gamepad2.right_stick_y);
      }
      
    
      //Wobble Goal Attachment
      if (gamepad2.y) {
        WobbleClamper.setPosition(0);
      } else if (gamepad2.b){
        WobbleClamper.setPosition(0.3);
      } else if (gamepad2.x) {
          RingClamper.setPosition(0.1);
      } else if (gamepad2.a) {
          RingClamper.setPosition(0.6);
      }
      
      //DcMotor Wobble Goal
      if (gamepad1.dpad_up) {
          RightBack.setPower(1);
        /*RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setTargetPosition(500);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setPower(0.75);*/

      } else if (gamepad1.dpad_down) {
        RightBack.setPower(-1);
          
        /*RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setTargetPosition(-500);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setPower(0.75);*/
      } 
      
      if (gamepad2.dpad_left) {
        Wobbler.setPower(1);
        
      } else if (gamepad2.dpad_right){
        Wobbler.setPower(-1);
      } else {
        Wobbler.setPower(0);
      }


      
      
      



        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.update();
      }
    }
  }
}
