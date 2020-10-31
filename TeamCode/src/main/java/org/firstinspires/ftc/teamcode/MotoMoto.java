package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MotoMoto", group = "")
public class MotoMoto extends LinearOpMode {

    private DcMotor Shooter, Intake, Conv, Wobble;
    private ElapsedTime runtime = new ElapsedTime();

    private double Scale (double Input) {
        return (double) Input * Math.abs(Input);
    }

    @Override
    public void runOpMode() {
        Wobble = hardwareMap.dcMotor.get("w");
        Conv = hardwareMap.dcMotor.get("c");
        Intake = hardwareMap.dcMotor.get("i");
        Shooter = hardwareMap.dcMotor.get("s");

//        String[] Moto = [Wobble, Conv, Intake, Shooter];

        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Conv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Wobble.setPower(0);
        Conv.setPower(0);
        Intake.setPower(0);
        Shooter.setPower(0);

        telemetry.addData(">", "INIT DONE");
        // RESET TIME
        runtime.reset();

        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (Math.abs(gamepad1.right_trigger) > 0.01 && gamepad1.a){
                    // MOVE SHOOTER
                    Shooter.setPower(0.7 * Scale(gamepad1.right_trigger));
                }  else if (Math.abs(gamepad1.left_trigger) > 0.01 && gamepad1.x){
                    // MOVE CONV
                    Conv.setPower(0.7 * Scale(gamepad1.right_trigger));
                } else if (Math.abs(gamepad1.left_trigger) > 0.01 && gamepad1.y){
                    // MOVE WOBBLE
                    Wobble.setPower(0.7 * Scale(gamepad1.right_trigger));
                } else if (gamepad1.dpad_down) {
                    // STOP MOTO
                    Wobble.setPower(0);
                    Conv.setPower(0);
                    Intake.setPower(0);
                    Shooter.setPower(0);
                } else if (gamepad1.dpad_left) {
                    // MOVE INTAKE
                    Intake.setPower(0.7);
                } else if (gamepad1.dpad_right) {
                    Intake.setPower(-0.7);
                }
                // Shooter, Intake, Conv, Wobble
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Shooter", Shooter.getPower());
                telemetry.addData("Intake", Intake.getPower());
                telemetry.addData("Conv", Conv.getPower());
                telemetry.addData("Wobble", Wobble.getPower());
                telemetry.update();
            }
        }
    }
}
