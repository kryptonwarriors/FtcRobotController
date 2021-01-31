package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderAuto", group = "")
public class EncoderAuto extends LinearOpMode {

    public DcMotor LeftForward, LeftBack, RightForward, RightBack;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");





        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //moveEncoders(FORWARD, 0.15, 2000);
            LeftForward.setPower(0.2);
            LeftBack.setPower(0.2);
            RightForward.setPower(0.2);
            RightBack.setPower(0.2);

            telemetry.addData("Status", "Program Complete");
            telemetry.addData("Position", LeftForward.getCurrentPosition() + "," + LeftForward.getTargetPosition());
            telemetry.update();

        }
    }

    public void moveEncoders(int Direction, double Power, int TargetPosition) {
        LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        if (Direction == FORWARD && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())){

            while (opModeIsActive() && !isStopRequested()) {
                LeftForward.setTargetPosition(TargetPosition);
                RightForward.setTargetPosition(TargetPosition);
                LeftBack.setTargetPosition(TargetPosition);
                RightBack.setTargetPosition(TargetPosition);

                LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                LeftForward.setPower(-Power);
                RightForward.setPower(Power);
                LeftBack.setPower(-Power);
                RightBack.setPower(Power);
            }

            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);


        } else if (Direction == BACKWARD && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())){

            while (opModeIsActive() && !isStopRequested()) {
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


        } else if (Direction == LEFT && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())){

            while (opModeIsActive() && !isStopRequested()) {
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


        } else if (Direction == RIGHT && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())){

            while (opModeIsActive() && !isStopRequested()) {
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


        }
    }
}
