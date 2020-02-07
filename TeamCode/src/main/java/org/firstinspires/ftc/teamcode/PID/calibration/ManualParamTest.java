package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.IMUBufferReader;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;
import java.util.List;
import java.lang.String;

import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.RUN_USING_ENCODER;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "ManualParamTest", group = "drive")
public class ManualParamTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 1000;
    private String TAG = "ManualParamTest";
    Localizer localizer = null;
    private HardwareMap hwMap = new HardwareMap(hardwareMap);


    private void sleep_millisec(int c)
    {
        try {
            Thread.sleep(c);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    //IMUBufferReader imu = IMUBufferReader.getSingle_instance(hardwareMap);
    private void grabStone(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
        sleep_millisec(400);

        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed);
        sleep_millisec(400);

        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);
        sleep_millisec(200);

        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);
        sleep_millisec(200);

    }
    private void transferReset() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosBlock);
            }
        };
    }
    private void initIntakeClaw() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);
                hwMap.clawServo2.setPosition(0.9336);

                sleep_millisec(2800);


                //hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block + 0.08);
                //resetLift(TeleopConstants.liftPower);
                sleep_millisec(300);

                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosRight);
                sleep_millisec(500);

                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosLeft);
                //intake(1);

                sleep_millisec(500);

                hwMap.innerTransfer.setPosition(TeleopConstants.intakeInitPosReset);
            }
        };

        Thread t = new Thread(){
            public void run(){
                //hwMap.clawServo2.setPosition(0.9336);
                sleep_millisec(2600);

                //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);

                sleep_millisec(600);


                hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);
            }
        };
        thread.start();
        t.start();
    }
    private void prepGrab(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
        sleep_millisec(200);

        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
        sleep_millisec(200);

        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
        sleep_millisec(200);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        localizer = drive.getLocalizer();
        waitForStart();

        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            transferReset();
            initIntakeClaw();
            init();
            prepGrab(FieldPosition.RED_QUARY);    //*******

        }

        VuforiaCamLocalizer vu = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCamLocalizer.VuforiaCameraChoice.PHONE_BACK);
        while (opModeIsActive()) {
            if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
                List<Double>  odo_positions = t.getWheelPositions();

                RobotLogger.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            List<Double> velocities = drive.getWheelVelocities();
            RobotLogger.dd(TAG, "velocities");
            drive.print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLogger.dd(TAG, "wheel positions");
            drive.print_list_double(positions);

            Pose2d pose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "Pose: x " + pose.getX());
            RobotLogger.dd(TAG, "Pose: y " + pose.getY());
            RobotLogger.dd(TAG, "Pose: heading " + Double.toString(pose.getHeading()));

            Pose2d vPose = vu.getPoseEstimate();
            RobotLogger.dd(TAG, "vuforia loc: " + vPose.toString());
            double v_double = DriveConstantsPID.getTeamCodePropertyValue("debug.ftc.grab");
            if (v_double != Double.MAX_VALUE) {
                int v_int = (int) v_double;
                if (v_int != 0) {
                    grabStone(FieldPosition.RED_QUARY);
                }
            }
            Thread.sleep(polling_interval);
        };

    }
}
