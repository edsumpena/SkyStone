package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.TensorflowDetector;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCameraChoice;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TensorFlowTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "TensorFlowTest";
    private SampleMecanumDriveBase _drive = null;
    private HardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        RobotLogger.dd(TAG, "unit test for TensorFlow skystone detector");

        waitForStart();

        if (isStopRequested()) return;
        TensorflowDetector vTester = null;

        int count = 0;
        while (!isStopRequested()) {
            RobotLogger.dd(TAG, "looop %d", count);
            vTester = TensorflowDetector.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_BACK);

            for (int i = 0; i < 10; i ++ ) {
                vTester.detectSkystone();
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            count ++;
            vTester.stop();

            vTester = TensorflowDetector.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_FRONT);

            for (int i = 0; i < 10; i ++ ) {
                vTester.detectSkystone();
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            count ++;
            vTester.stop();

        }

        RobotLogger.dd(TAG, "----------done --------------------- unit test for tensor flow");
    }
}
