package org.firstinspires.ftc.teamcode.Tensorflow;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Detect;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Arrays;
import java.util.List;

/**
 * There are currently 4 available models:
 *  -"Skystone.tflite" --Standard trained, Skystone & stone detecting model provided by FTC
 *  -"skystoneTFOD_v1_[50-15].tflite" --Basic retrained Skystone & Stone detection model
 *  -"skystoneTFOD_v2_[105-15].tflite" --Training set better optimized for different light conditions
 *  -"skystoneTFOD_v3_[150-30].tflite" --Training set better optimized for detection at longer distances
 *  -"skystoneTFOD_v4_[160-30].tflite" --Training set contains more images at longer distances
 *
 *  Skystone file name format: "skystoneTFOD_vVersion#_[# of training images-# of testing images].tflite
 *  NOTE: Covering a wider variety of light conditions sacrifices some accuracy
 */

@TeleOp(name = "RetrainedSkystoneDetection", group = "Linear Opmode")
//@Disabled
public class CustomTensorFlowObjectDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";  //"skystoneTFOD_v2_[105-15].tflite";    //Set model (see above for file names)
    private static final String LABEL_FIRST_ELEMENT = "skystone";   //"skystone";
    private static final String LABEL_SECOND_ELEMENT = "stone";  //"stone";
    private static final String VUFORIA_KEY =
            "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";  //Variable Place--Remember to insert key here

    private VuforiaLocalizer vuforia;       //Image dimensions 360x640
    private static double imgWidth = 1.0;

    private TFObjectDetector tfod;

    int imgHeight = 0;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        TFODCalc.init();
        TFODCalc.getPhoneCamConstants();

        try {
            initVuforia();
        } catch (Exception e){
            e.printStackTrace();
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        Detect detect = new Detect();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Focal Point", TFODCalc.getFocalLength());
        telemetry.addData("Sensor Height", TFODCalc.getSensorHeight());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        int objIndex = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            double objWidthpx = Math.abs(recognition.getTop() - recognition.getBottom());
                            double objHeightpx = Math.abs(recognition.getLeft() - recognition.getRight());
                            double distanceToObj = TFODCalc.getDistanceToObj(127,
                                    imgHeight, objHeightpx);

                            telemetry.addData("**Height", objHeightpx);
                            telemetry.addData("**Width", objWidthpx);
                            telemetry.addData("**Distance to Object", distanceToObj);
                            telemetry.addData("Position",
                                    Arrays.toString(detect.getSkystonePositionsBlue(updatedRecognitions, imgWidth)));
                            telemetry.addData("","----------------------------");

                            objIndex += 1;
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() throws InterruptedException {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue

        imgHeight = frame.getImage(0).getHeight();
        imgWidth = frame.getImage(0).getWidth();

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.75;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
