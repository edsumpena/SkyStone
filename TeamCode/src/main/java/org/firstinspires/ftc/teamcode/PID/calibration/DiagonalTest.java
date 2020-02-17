package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "DiagonalTest", group = "drive")
//@Disabled
public class DiagonalTest extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "DiagonalTest";
    SampleMecanumDriveBase _drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;

        if (_drive == null) {
            if (DriveConstantsPID.USING_BULK_READ == false)
                _drive = new SampleMecanumDriveREV(hardwareMap, DriveConstantsPID.USING_STRAFE_DIAGONAL);
            else
                _drive = new SampleMecanumDriveREVOptimized(hardwareMap, DriveConstantsPID.USING_STRAFE_DIAGONAL);

            _drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
            _drive.setPoseEstimate(new Pose2d(0, 0, _drive.getExternalHeading()));
        }

        waitForStart();

        while (!isStopRequested()) {
            //_drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));

            if (DriveConstantsPID.DIAGONAL_SPLIT)
                Path.StrafeDiagonalHelper(_drive, new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0));
            else {
                Trajectory trajectory = _drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0))
                        .build();
                _drive.followTrajectorySync(trajectory);
            }

            Localizer localizer = _drive.getLocalizer();
            if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer != null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer) localizer; // @TODO
                List<Double> odo_positions = t.getWheelPositions();

                RobotLogger.dd(TAG, "odometry positions");
                _drive.print_list_double(odo_positions);
            }

            List<Double> positions = _drive.getWheelPositions();
            RobotLogger.dd(TAG, "wheel positions");
            _drive.print_list_double(positions);

            Pose2d currentPos = _drive.getPoseEstimate();
            Pose2d error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
            //drive.turnSync(Math.toRadians(90));
            Path.sleep_millisec_opmode(2000, this);

            if (DriveConstantsPID.RESET_FOLLOWER)
                _drive.resetFollowerWithParameters(DriveConstantsPID.USING_STRAFE_DIAGONAL, false);

            if (DriveConstantsPID.DIAGONAL_SPLIT)
                Path.StrafeDiagonalHelper(_drive, new Vector2d(0, 0));
            else {
                Trajectory trajectory = _drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(0, 0))
                        .build();
                _drive.followTrajectorySync(trajectory);
            }

            currentPos = _drive.getPoseEstimate();
            error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
            if (DriveConstantsPID.RESET_FOLLOWER)
                _drive.resetFollowerWithParameters(DriveConstantsPID.USING_STRAFE_DIAGONAL, false);

            Path.sleep_millisec_opmode(5000, this);
        }
    }
}
