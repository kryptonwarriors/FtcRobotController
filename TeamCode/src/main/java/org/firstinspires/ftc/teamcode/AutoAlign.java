package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;


import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;

@Autonomous(name = "AutoAlign", group = "Test")
public class AutoAlign extends LinearOpMode {
    public DcMotor LeftForward, LeftBack, RightForward, RightBack, Wobbler, Ringer;
    public Servo WobbleClamper, RingClamper;
    public DistanceSensor BackDistance, RightDistance;
    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int LTURN = 4;
    int RTURN = 5;

    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    private Orientation angles;
    public Orientation lastAngle = new Orientation();

    double inchesToVerticalAlignment = 64;
    double inchesToHorizontalAlignment = 31;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        Wobbler = hardwareMap.dcMotor.get("Wobbler");
        Ringer = hardwareMap.dcMotor.get("Ringer");

        WobbleClamper = hardwareMap.servo.get("WobbleClamper");
        RingClamper = hardwareMap.servo.get("RingClamper");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");

        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

    }
/*
    //TODO create conversion from distance measured initially and convert to encoders needed to move
    public void align(double power) {
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && (BackDistance.getDistance(DistanceUnit.INCH) < inchesToVerticalAlignment)) {
            correction = pidDrive.performPID(getAngle());

            LeftForward.setPower(power + correction);
            LeftBack.setPower(power + correction);
            RightForward.setPower(power - correction);
            RightBack.setPower(power - correction);

            telemetry.addData("correction", correction);
            telemetry.addData("LeftForward", LeftForward.getPower());
            telemetry.addData("RightForward", RightForward.getPower());
            telemetry.addData("LeftBack", LeftBack.getPower());
            telemetry.addData("RightBack", RightBack.getPower());
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        if (RightDistance.getDistance(DistanceUnit.INCH) < inchesToHorizontalAlignment){
            //Strafe Left to desired Horizontal Location
            while (opModeIsActive() && (BackDistance.getDistance(DistanceUnit.INCH) < inchesToHorizontalAlignment)) {
                correction = pidDrive.performPID(getAngle());

                LeftForward.setPower(power + correction);
                LeftBack.setPower(power + correction);
                RightForward.setPower(power - correction);
                RightBack.setPower(power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (RightDistance.getDistance(DistanceUnit.INCH) > inchesToHorizontalAlignment){
            //Strafe Right to desired Horizontal Location
            while (opModeIsActive() && (BackDistance.getDistance(DistanceUnit.INCH) > inchesToHorizontalAlignment)) {
                correction = pidDrive.performPID(getAngle());

                LeftForward.setPower(power + correction);
                LeftBack.setPower(power + correction);
                RightForward.setPower(power - correction);
                RightBack.setPower(power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }
        LeftForward.setPower(0);
        RightForward.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }
    */
}


