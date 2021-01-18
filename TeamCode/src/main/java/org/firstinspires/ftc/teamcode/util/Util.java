package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.meetAuto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Util {
    // SERVO
    public Servo WobbleClamper, RingClamper;
    // DC MOTORS
    private static DcMotor LeftForward;
    private static DcMotor LeftBack;
    private static DcMotor RightForward;
    private static DcMotor RightBack;




    // SENSORS
    private static BNO055IMU imu;

    public DistanceSensor BackDistance, RightDistance, FrontDistance, LeftDistance;

    private static HardwareMap hardwareMap;
    private int BACKWARD = 1;
    private int LEFT = 2;
    private int RIGHT = 3;
    private int RTurn = 6;
    private int LTurn = 7;
    private int FORWARD = 0;


    public Util(ElapsedTime runtime, HardwareMap hardwareMap) {

        /* SERVO */

        /* DC MOTORS */
        LeftForward = hardwareMap.get ( DcMotor.class, "LeftForward" );
        RightForward = hardwareMap.get ( DcMotor.class, "RightForward" );
        LeftBack = hardwareMap.get ( DcMotor.class, "LeftBack" );
        RightBack = hardwareMap.get ( DcMotor.class, "RightBack" );

        /* SENSORS */
        imu = hardwareMap.get ( BNO055IMU.class, "imu" );

        LeftForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        LeftBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
    }

    public void StopDrive() {
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
    public void InitHardwareMap() {
        imu = hardwareMap.get( BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30,250);

        static final int REGION_WIDTH = 95;
        static final int REGION_HEIGHT = 70;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile meetAuto.RingDeterminationPipeline.RingConfiguration configuration = meetAuto.RingDeterminationPipeline.RingConfiguration.C;

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

            configuration = meetAuto.RingDeterminationPipeline.RingConfiguration.C; // Record our configuration

            if(avg1 > FOUR_RING_THRESHOLD){
                configuration = meetAuto.RingDeterminationPipeline.RingConfiguration.C;
            }else if (avg1 > ONE_RING_THRESHOLD){
                configuration = meetAuto.RingDeterminationPipeline.RingConfiguration.B;
            }else{
                configuration = meetAuto.RingDeterminationPipeline.RingConfiguration.A;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    5);

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }


    // TODO - Test with Sensors.
}