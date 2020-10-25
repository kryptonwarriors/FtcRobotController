package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "opencvSkystoneDetector", group = "Sky autonomous")

//@Disabled//comment out this line before using
public class opencvSkystoneDetector extends LinearOpMode {
    private static float rectHeight = .6f / 8f;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255


    private static int valBottom = -1;
    private static int valTop = -1;
    private static String ringConfig;
    private static float rectWidth = 1.5f / 7f;
    private static float offsetX = 0f / 8f;
    //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = -.3f / 8f;
    //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] bottomPos = {4f / 8f + offsetX, 3.9f / 8f + offsetY};//0 = col, 1 = row
    private static float[] topPos = {4f / 8f + offsetX, 3.6f / 8f + offsetY};
    private ElapsedTime runtime = new ElapsedTime();
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private static DcMotor LeftForward = null;
    private static DcMotor LeftBack = null;
    private static DcMotor RightForward = null;
    private static DcMotor RightBack = null;

    private static DcMotor Wobbler = null;

    private static Servo WobbleClamper = null;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {


        //int cameraMonitorViewId =
                //hardwareMap.appContext.getResources ().getIdentifier ( "cameraMonitorViewId", "id",
                                        //                               hardwareMap.appContext.getPackageName () );

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
       /* webcam = OpenCvCameraFactory.getInstance ().createWebcam (
                hardwareMap.get ( WebcamName.class, "Webcam 1" ),
                cameraMonitorViewId );        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this
*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance ().createWebcam (
                hardwareMap.get ( WebcamName.class, "Webcam 1" ),
                cameraMonitorViewId );
        webcam.openCameraDevice ();//open camera
        webcam.setPipeline ( new StageSwitchingPipeline () );//different stages
        webcam.startStreaming ( rows, cols, OpenCvCameraRotation.UPRIGHT );//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart ();
        runtime.reset ();


        while (!isStopRequested ()) {

            if (valBottom != 0) {
                if (valTop != 0) {
                    ringConfig = "C";
                } else{
                    ringConfig = "B";
                }
            } else {
                ringConfig = "A";
            }

            telemetry.addData ( "Values", valTop + "   " + valBottom + "   ");
            telemetry.addData ( "ringConfig", ringConfig );
            //telemetry.addData ( "Height", rows );
            //telemetry.addData ( "Width", cols );

            telemetry.update ();


        }
    }



    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<> ();
        private Stage[] stages = Stage.values ();

        private Stage stageToRenderToViewport = Stage.detection;

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal ();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear ();
            /*
             * This pipeline finds the contours of yellow blobs
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor ( input, yCbCrChan2Mat,
                               Imgproc.COLOR_RGB2YCrCb );//converts rgb to ycrcb
            Core.extractChannel ( yCbCrChan2Mat, yCbCrChan2Mat, 2 );//takes cb difference and stores

            //b&w
            Imgproc.threshold ( yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV );

            //outline/contour
            Imgproc.findContours ( thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST,
                                   Imgproc.CHAIN_APPROX_SIMPLE );
            yCbCrChan2Mat.copyTo ( all );//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixBottom = thresholdMat.get ( (int) (input.rows () * bottomPos[1]),
                                                 (int) (input.cols () * bottomPos[0]) );//gets value at circle
            valBottom = (int) pixBottom[0];

            double[] pixTop = thresholdMat.get ( (int) (input.rows () * topPos[1]),
                                                  (int) (input.cols () * topPos[0]) );//gets value at circle
            valTop = (int) pixTop[0];


            //create three points
            Point pointBottom = new Point( (int) (input.cols () * bottomPos[0]),
                                         (int) (input.rows () * bottomPos[1]) );
            Point pointTop = new Point( (int) (input.cols () * topPos[0]),
                                          (int) (input.rows () * topPos[1]) );

            //draw circles on those points
            Imgproc.circle ( all, pointBottom, 5, new Scalar( 255, 0, 0 ), 1 );//draws circle
            Imgproc.circle ( all, pointTop, 5, new Scalar( 255, 0, 0 ), 1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle (//1-3
                               all,
                               new Point(
                                       input.cols () * (topPos[0] - rectWidth / 2),
                                       input.rows () * (topPos[1] - rectHeight / 2) ),
                               new Point(
                                       input.cols () * (topPos[0] + rectWidth / 2),
                                       input.rows () * (topPos[1] + rectHeight / 2) ),
                               new Scalar( 0, 255, 0 ), 3 );
            Imgproc.rectangle (//3-5
                               all,
                               new Point(
                                       input.cols () * (bottomPos[0] - rectWidth / 2),
                                       input.rows () * (bottomPos[1] - rectHeight / 2) ),
                               new Point(
                                       input.cols () * (bottomPos[0] + rectWidth / 2),
                                       input.rows () * (bottomPos[1] + rectHeight / 2) ),
                               new Scalar( 0, 255, 0 ), 3 );

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }
}