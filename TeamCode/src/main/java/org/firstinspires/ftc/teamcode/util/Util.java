package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Util {
    // SERVO
    // DC MOTORS
    private static DcMotor LeftForward;
    private static DcMotor LeftBack;
    private static DcMotor RightForward;
    private static DcMotor RightBack;
    // SENSORS
    private static BNO055IMU IMU;

    private static HardwareMap hardwareMap;
    private int BACKWARD = 1;
    private int LEFT = 2;
    private int RIGHT = 3;
    private int RTurn = 6;
    private int LTurn = 7;
    private int FORWARD = 0;


    Util(ElapsedTime runtime, HardwareMap hardwareMap) {

        /* SERVO */

        /* DC MOTORS */
        LeftForward = hardwareMap.get ( DcMotor.class, "LeftForward" );
        RightForward = hardwareMap.get ( DcMotor.class, "RightForward" );
        LeftBack = hardwareMap.get ( DcMotor.class, "LeftBack" );
        RightBack = hardwareMap.get ( DcMotor.class, "RightBack" );

        /* SENSORS */
        IMU = hardwareMap.get ( BNO055IMU.class, "IMU" );

        LeftForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        LeftBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
    }

    private void StopDrive() {
        LeftBack.setPower ( 0.0 );
        LeftForward.setPower ( 0.0 );
        RightForward.setPower ( 0.0 );
        RightBack.setPower ( 0.0 );
    }

    // Direction = FORWARD || BACKWARD || LEFT || RIGHT || RTurn | LTurn
    // Power = -1 to 1
    // Duration = x > 0 in ms
    // TODO - Implement Duration
    private void MoveByTime(int Direction, double Power, double Duration) {

        LeftForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        LeftForward.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        if (Direction == FORWARD) {
            RightForward.setPower ( Power );
            LeftBack.setPower ( -Power );
            LeftForward.setPower ( -Power );
            RightBack.setPower ( Power );
        } else if (Direction == BACKWARD) {
            RightForward.setPower ( -Power );
            LeftBack.setPower ( Power );
            LeftForward.setPower ( Power );
            RightBack.setPower ( -Power );
        } else if (Direction == LEFT) {
            LeftForward.setPower ( Power );
            LeftBack.setPower ( -Power );
            RightForward.setPower ( Power );
            RightBack.setPower ( -Power );
        } else if (Direction == RIGHT) {
            LeftForward.setPower ( -Power );
            LeftBack.setPower ( Power );
            RightForward.setPower ( -Power );
            RightBack.setPower ( Power );
        } else if (Direction == RTurn) {
            LeftForward.setPower ( -Power );
            LeftBack.setPower ( -Power );
            RightForward.setPower ( -Power );
            RightBack.setPower ( -Power );
        } else if (Direction == LTurn) {
            LeftForward.setPower ( Power );
            LeftBack.setPower ( Power );
            RightForward.setPower ( Power );
            RightBack.setPower ( Power );
        }
    }

    // TODO - Move Constructor Code here.
    // TODO - IMU Calibration
    // TODO - Added Senors and make it work from main auto.
    // TODO - Reverse the 2 Motors
    private void InitHardwareMap() {
        IMU = hardwareMap.get( BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        IMU.initialize(imuParameters);

        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");
    }

    // TODO - Implement While Loop To Check Encoder Target Pos
    // TODO - Correct Positive and Negative Values to Move.
    private void MoveWithEncoders(int Direction, double Power, int targetPos) {
        LeftForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        LeftForward.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );

        if (Direction == FORWARD) {
            RightForward.setTargetPosition ( targetPos );
            LeftBack.setTargetPosition ( -targetPos );
            LeftForward.setTargetPosition ( -targetPos );
            RightBack.setTargetPosition ( targetPos );
        } else if (Direction == BACKWARD) {
            RightForward.setTargetPosition ( -targetPos );
            LeftBack.setTargetPosition ( targetPos );
            LeftForward.setTargetPosition ( targetPos );
            RightBack.setTargetPosition ( -targetPos );
        } else if (Direction == LEFT) {
            LeftForward.setTargetPosition ( targetPos );
            LeftBack.setTargetPosition ( -targetPos );
            RightForward.setTargetPosition ( targetPos );
            RightBack.setTargetPosition ( -targetPos );
        } else if (Direction == RIGHT) {
            LeftForward.setTargetPosition ( -targetPos );
            LeftBack.setTargetPosition ( targetPos );
            RightForward.setTargetPosition ( -targetPos );
            RightBack.setTargetPosition ( targetPos );
        } else if (Direction == RTurn) {
            LeftForward.setTargetPosition ( -targetPos );
            LeftBack.setTargetPosition ( -targetPos );
            RightForward.setTargetPosition ( -targetPos );
            RightBack.setTargetPosition ( -targetPos );
        } else if (Direction == LTurn) {
            LeftForward.setTargetPosition ( targetPos );
            LeftBack.setTargetPosition ( targetPos );
            RightForward.setTargetPosition ( targetPos );
            RightBack.setTargetPosition ( targetPos );
        }
    }

    // TODO - Test with Sensors.
}