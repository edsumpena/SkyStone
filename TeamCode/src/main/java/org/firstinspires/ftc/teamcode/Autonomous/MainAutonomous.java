package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Detect;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.TensorflowDetector;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCameraChoice;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Autonomous", group = "LinearOpMode")
//@Disabled
public class MainAutonomous extends LinearOpMode {
    private FieldPosition fieldPosition = null;
    private HardwareMap hwMap;
    private double imgWidth;
    private int[] skystonePositions;
    private Pose2d startingPos;
    private Path path;
    private SampleMecanumDriveBase straightDrive;
    private SampleMecanumDriveBase strafeDrive;
    private boolean initialize = false;
    public BNO055IMU imu;
    private VuforiaCamLocalizer vuLocalizer = null;
    private VuforiaCameraChoice camChoice;
    private TensorflowDetector tfDetector;

    @Override
    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);

        FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        // Setup drive, lift, and intake motors
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DriveConstantsPID.updateConstantsFromProperties();
        camChoice = VuforiaCameraChoice.HUB_USB;  // default one;
        while (!isStarted() && !isStopRequested()) {
            // Select starting position from user input
            if (fieldPosition == null) {
                telemetry.addData("SELECT STARTING LOCATION", "Press one of the following buttons below to " +
                        "select the autonomous starting position. Once you have selected, press the \"start\" button " +
                        "on gamepad A.");
                telemetry.addData("A", FieldPosition.BLUE_FOUNDATION_PARK);
                telemetry.addData("B", FieldPosition.RED_FOUNDATION_PARK);
                telemetry.addData("Y", FieldPosition.RED_QUARY);
                telemetry.addData("X", FieldPosition.BLUE_QUARY);
            }

            if(!gamepad1.start) {
                if (gamepad1.a) {
                    fieldPosition = FieldPosition.BLUE_FOUNDATION_PARK;
                    startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));
                } else if (gamepad1.b) {
                    fieldPosition = FieldPosition.RED_FOUNDATION_PARK;
                    startingPos = new Pose2d(new Vector2d(20.736, -63.936), Math.toRadians(90));
                } else if (gamepad1.y) {
                    fieldPosition = FieldPosition.RED_QUARY;
                    startingPos = new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0));
                    //camChoice = VuforiaCameraChoice.PHONE_FRONT;  // default one;
                } else if (gamepad1.x) {
                    fieldPosition = FieldPosition.BLUE_QUARY;
                    startingPos = new Pose2d(new Vector2d(-34.752, 63.936), Math.toRadians(0));
                    //camChoice = VuforiaCameraChoice.PHONE_FRONT;  // default one;
                } else if(gamepad1.right_bumper){
                    fieldPosition = FieldPosition.BLUE_FOUNDATION_DRAG;
                    startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));
                } else if(gamepad1.left_bumper){
                    fieldPosition = FieldPosition.RED_FOUNDATION_DRAG;
                    startingPos = new Pose2d(new Vector2d(20.736, -63.936), Math.toRadians(90));
                }
            }

            if (fieldPosition != null && !initialize)
                telemetry.addData("SELECTED STARTING LOCATION", fieldPosition);

            if (gamepad1.start && fieldPosition != null)
                initialize = true;

            if (initialize) {
                telemetry.addData("STATUS", "Calibrating IMU...");
                telemetry.update();

                if (DriveConstantsPID.USING_BULK_READ == false) {
                    straightDrive = new SampleMecanumDriveREV(hardwareMap, false);
                    strafeDrive = new SampleMecanumDriveREV(hardwareMap, true);
                }
                else {
                    straightDrive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
                    strafeDrive = new SampleMecanumDriveREVOptimized(hardwareMap, true);
                }
                /*
                strafeDrive = new SampleMecanumDriveREV(hardwareMap, true);
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
                */
                telemetry.addData("STATUS", "Done!");
                telemetry.update();
                path = new Path(hwMap, this, straightDrive, strafeDrive, hardwareMap, imu, telemetry);

                if (fieldPosition == FieldPosition.RED_QUARY || fieldPosition == FieldPosition.BLUE_QUARY) {
                    telemetry.addData("STATUS", "Initializing TensorFlow...");
                    telemetry.update();

                    //drivetrain.resetEncoders();

                    //if(fieldPosition == FieldPosition.RED_QUARY)

                    tfDetector = TensorflowDetector.getSingle_instance(hardwareMap, camChoice);

                    //else if(fieldPosition == FieldPosition.BLUE_QUARY)
                    //    initVuforia(CameraController.PHONECAM);

                    DriveConstant.writeSerializedObject(AppUtil.ROOT_FOLDER + "/FIRST/PrevRunPath.txt", fieldPosition.toString());

                    telemetry.addData("STATUS", "Done!");
                    telemetry.update();
                }
                break;
            }
            telemetry.update();
        }

        //drivetrain.resetEncoders();

        // begin tfod processing before starting -- use it to ascertain the positions of skystones in quarry
        while (!isStarted() && (fieldPosition == FieldPosition.BLUE_QUARY || fieldPosition == FieldPosition.RED_QUARY) &&
                !isStopRequested()) {
            List<Recognition> recognized = tfDetector.recognize();

            if (recognized != null && fieldPosition == FieldPosition.BLUE_QUARY)
                try {
                    skystonePositions = Detect.getSkystonePositionsBlue(recognized, imgWidth);
                } catch (Exception e){ e.printStackTrace(); }
            else if (recognized != null && fieldPosition == FieldPosition.RED_QUARY)
                try{
                    skystonePositions = Detect.getSkystonePositionsRed(recognized, imgWidth);
                } catch (Exception e){ e.printStackTrace(); }

            telemetry.addData("SKYSTONE POSITIONS", Arrays.toString(skystonePositions));
            telemetry.addData("External Heading",
                    Math.round(Math.toDegrees(straightDrive.getExternalHeading()) * 1000.0) / 1000.0);
            telemetry.addData("Current (starting) Location", path.getPoseEstimate());
            telemetry.update();
            try {
                Thread.sleep(200);
            } catch (Exception e) {
            }
        }

        waitForStart();
        if (tfDetector != null) {
            RobotLogger.dd("", "to shutdown tensor flow");
            tfDetector.stop();
            tfDetector = null;
            RobotLogger.dd("", "tensor flow is shutdown");
        }

        if (DriveConstantsPID.USE_VUFORIA_LOCALIZER) {
            vuLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap,
                    VuforiaCameraChoice.PHONE_BACK, true);
        }

        if (opModeIsActive() && fieldPosition != null) {
            if(skystonePositions != null || fieldPosition == FieldPosition.RED_FOUNDATION_PARK || fieldPosition == FieldPosition.BLUE_FOUNDATION_PARK) {
                sendData();
                //resetLiftEncoder();
                switch (fieldPosition) {
                    case RED_QUARY:
                        if(skystonePositions[0] > 3)
                            skystonePositions = new int[]{3, 6};
                        path.RedQuary(skystonePositions, vuLocalizer);
                        break;
                    case RED_FOUNDATION_PARK:
                        path.RedFoundationPark();
                        break;
                    case BLUE_QUARY:
                        if(skystonePositions[0] > 3)
                            skystonePositions = new int[]{3, 6};
                        path.BlueQuary(skystonePositions, vuLocalizer);
                        break;
                    case BLUE_FOUNDATION_PARK:
                        path.BlueFoundationPark();
                        break;
                    case BLUE_FOUNDATION_DRAG:
                        path.BlueFoundationDrag();
                        break;
                    case RED_FOUNDATION_DRAG:
                        break;
                }
            } else {
                while (opModeIsActive()) {
                    telemetry.addData("ERROR", "No SKYSTONE position data received!");
                    telemetry.update();
                }
            }
        } else {
            while (opModeIsActive()) {
                telemetry.addData("ERROR", "Please select parameters with gamepad1 upon initialization!");
                telemetry.update();
            }
        }

    }



    private void sendData() {
        // inject imu heading and tfod recognition into currently running path from another thread

        Thread update = new Thread() {
            public void run() {
                while (opModeIsActive() && (tfDetector != null)) {
                    path.updateTFODData(tfDetector.recognize());
                    path.updateHeading();
                    try {
                        Thread.sleep(500);
                    } catch (Exception e) {
                    }
                    if (isStopRequested() && tfDetector != null)
                        tfDetector.stop();
                }
            }
        };
        update.start();
    }

    private void resetLiftEncoder() {
        Thread resetEncoderLoop = new Thread() {
            public void run() {
                while (opModeIsActive())
                    if (!hwMap.liftReset.getState()) {
                        hwMap.liftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        try {
                            Thread.sleep(300);
                        } catch (Exception e) {
                        }
                        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
            }
        };
        resetEncoderLoop.start();
    }
}
