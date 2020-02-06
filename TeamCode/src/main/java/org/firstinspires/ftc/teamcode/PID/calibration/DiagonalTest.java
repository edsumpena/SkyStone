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

    // input xy: absolute location of destination
    private void StrafeDiagonalHelper(SampleMecanumDriveBase _drive, Vector2d dest) {
        Trajectory trajectory;
        Pose2d currentPos = _drive.getPoseEstimate();
        TrajectoryBuilder  builder = null;
        if (DriveConstantsPID.USING_STRAFE_DIAGNAL)
            builder = new TrajectoryBuilder(currentPos, DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        else
            builder = new TrajectoryBuilder(currentPos, DriveConstantsPID.BASE_CONSTRAINTS);

        Pose2d error_pose = _drive.follower.getLastError();

        double current_x = currentPos.getX();
        double current_y = currentPos.getY();
        double delta_x = dest.getX() - current_x;
        double delta_y = dest.getY() - current_y;

        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, xy: %s", dest.toString());
        Vector2d firstStop;
        if (Math.abs(delta_x) > Math.abs(delta_y)) {
            RobotLogger.dd(TAG, "x > y, line first and then strafe");
            double square_offset = Math.abs(delta_y);
            double new_x = 0;
            if (delta_x > 0)
                new_x = dest.getX() - square_offset;
            else
                new_x = dest.getX() + square_offset;

            firstStop = new Vector2d(new_x, current_y);
            RobotLogger.dd(TAG, "added one line to stop: " + firstStop.toString());
            builder.setReversed(false).lineTo(firstStop).strafeTo(dest);
        }
        else if (Math.abs(delta_x) < Math.abs(delta_y)){
            RobotLogger.dd(TAG, "x > y, strafe first and then line");
            double square_offset = Math.abs(delta_x);
            double new_y = 0;
            if (delta_y < 0)
                new_y = current_y - square_offset;
            else
                new_y = current_y + square_offset;

            firstStop = new Vector2d(dest.getX(), new_y);
            RobotLogger.dd(TAG, "added one strafe stop: " + firstStop.toString());
            builder.setReversed(false).strafeTo(firstStop).strafeTo(dest);
        }
        else
        {
            //double y_offset = delta_y - delta_x;
            builder.setReversed(false).strafeTo(dest);
        }
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);

        currentPos = _drive.getPoseEstimate();
        error_pose = _drive.follower.getLastError();
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
    }
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;

        waitForStart();

        if (isStopRequested()) return;

        if (_drive == null) {
            if (DriveConstantsPID.USING_BULK_READ == false)
                _drive = new SampleMecanumDriveREV(hardwareMap, true);
            else
                _drive = new SampleMecanumDriveREVOptimized(hardwareMap, true);
            _drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
            _drive.setPoseEstimate(new Pose2d(0, 0, _drive.getExternalHeading()));
        }

        StrafeDiagonalHelper(_drive, new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0));
        /*
        Trajectory trajectory = _drive.trajectoryBuilder()
                .strafeTo(new Vector2d(DriveConstantsPID.TEST_DISTANCE, DriveConstantsPID.TEST_DISTANCE_0))
                .build();
        _drive.followTrajectorySync(trajectory);
        */
        Localizer localizer = _drive.getLocalizer();
        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLogger.dd(TAG, "odometry positions");
            _drive.print_list_double(odo_positions);
        }

        List<Double> positions = _drive.getWheelPositions();
        RobotLogger.dd(TAG, "wheel positions");
        _drive.print_list_double(positions);
    }
}
