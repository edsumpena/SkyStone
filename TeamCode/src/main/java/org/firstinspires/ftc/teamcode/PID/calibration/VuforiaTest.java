package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class VuforiaTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "VuforiaTest";
    private SampleMecanumDriveBase _drive = null;
    private HardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        RobotLogger.dd(TAG, "unit test for vuforia localizer");

        waitForStart();

        if (isStopRequested()) return;
        VuforiaCamLocalizer vLocalizer = null;

        int count = 0;
        while (!isStopRequested()) {
            vLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCamLocalizer.VuforiaCameraChoice.PHONE_BACK);

            Pose2d poseEstimate;
            for (int i = 0; i < 10; i ++ ) {
                poseEstimate = vLocalizer.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
                RobotLogger.dd(TAG, "vuforia localization: " + poseEstimate.toString());
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            count ++;
            //vLocalizer.stop();
        }

        RobotLogger.dd(TAG, "----------done --------------------- unit test for vuforia localizer");
    }
}
