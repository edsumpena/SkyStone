
package org.firstinspires.ftc.teamcode.PID.calibration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "ManualStrafeTest", group = "drive")
@Disabled
public class ManualStrafeTest extends LinearOpMode {
    public static double DISTANCE = DriveConstantsPID.TEST_DISTANCE;
    private String TAG = "ManualStrafeTest";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstantsPID.updateConstantsFromProperties();
        DISTANCE = DriveConstantsPID.TEST_DISTANCE;
        SampleMecanumDriveBase drive = null;
        if (DriveConstantsPID.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, DriveConstantsPID.USING_STRAFE_DIAGONAL);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, DriveConstantsPID.USING_STRAFE_DIAGONAL);

        drive.setBrakeonZeroPower(DriveConstantsPID.BRAKE_ON_ZERO);
        RobotLog.dd(TAG, "trajectoryBuilder forward, DISTANCE: "+Double.toString(DISTANCE));

        waitForStart();

        if (isStopRequested()) return;

        DriveConstantsPID.strafeDistance(hardwareMap, DISTANCE, true);

        Localizer localizer = drive.getLocalizer();
        if (DriveConstantsPID.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLog.dd(TAG, "odometry positions");
            drive.print_list_double(odo_positions);
        }

        List<Double> positions = drive.getWheelPositions();
        RobotLog.dd(TAG, "wheel positions");
        drive.print_list_double(positions);
    }
}
