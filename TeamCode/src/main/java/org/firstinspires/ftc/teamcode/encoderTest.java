package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "encoderTest", group = "")
public class encoderTest extends LinearOpMode{

    public DcMotor LeftForward, LeftBack, RightForward, RightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Left Wheel", RightBack.getCurrentPosition());
            telemetry.addData("Right Wheel", LeftForward.getCurrentPosition());
            telemetry.addData("Center Wheel", RightForward.getCurrentPosition());
            telemetry.update();

        }
    }
}
