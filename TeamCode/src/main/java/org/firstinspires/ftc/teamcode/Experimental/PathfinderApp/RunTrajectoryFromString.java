package org.firstinspires.ftc.teamcode.Experimental.PathfinderApp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.ArrayList;
import java.util.UnknownFormatConversionException;

public class RunTrajectoryFromString {
    private ArrayList<Pose2d> data;
    private ArrayList<MoveOptions> options;
    private SampleMecanumDriveBase rev;
    private TrajectoryBuilder builder;
    private Trajectory trajectory;

    public RunTrajectoryFromString(SampleMecanumDriveBase drive){
        rev = drive;
    }

    public void runTrajectory(String traj){
        try{
            data = TrajectoryStringConverter.generateP2dFromString(traj);
            options = TrajectoryStringConverter.generateMoveOptionsFromString(traj);
        } catch (Exception e){
            throw new UnknownFormatConversionException("Failed to convert string to trajectory!");
        }
        rev.getLocalizer().setPoseEstimate(data.get(0));

        for(int i = 1; i < data.size(); i++){
            DriveBuilderReset(options.get(i) == MoveOptions.StrafeTo);
            switch(options.get(i)){
                case LineTo_Forward:
                    builder.setReversed(false).lineTo(new Vector2d(data.get(i).getX(), data.get(i).getY()));
                    break;
                case LineTo_Reversed:
                    builder.setReversed(true).lineTo(new Vector2d(data.get(i).getX(), data.get(i).getY()));
                    break;
                case StrafeTo:
                    builder.setReversed(false).strafeTo(new Vector2d(data.get(i).getX(), data.get(i).getY()));
                    break;
                case SplineTo_Forward:
                    builder.setReversed(false).splineTo(data.get(i));
                    break;
                case SplineTo_Reversed:
                    builder.setReversed(true).splineTo(data.get(i));
                    break;
            }
            trajectory = builder.build();   //x - 2.812, y + 7.984
                rev.followTrajectorySync(trajectory);
        }
    }

    private void DriveBuilderReset(boolean isStrafe) {
            Pose2d currentPos = rev.getPoseEstimate();
            //Pose2d newPos = currentPos;
            //Pose2d error_pose = revBulk.follower.getLastError();
            //RobotLog.dd("TrajectoryRunner", "start new step: %s, count[%d], currentPos %s, errorPos %s",
            //        label, step_count, currentPos.toString(), error_pose.toString());

            rev.resetFollowerWithParameters(isStrafe, false);

            //_drive = new SampleMecanumDriveREV(hardwareMap, isStrafe, init_imu);
            rev.getLocalizer().setPoseEstimate(currentPos);
            rev.getLocalizer().update();
            if (!isStrafe) {
                builder = new TrajectoryBuilder(rev.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
            } else {
                builder = new TrajectoryBuilder(rev.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
            }
            RobotLog.dd("TrajectoryRunner", "drive and builder created, initialized with pose: " + rev.getPoseEstimate().toString());
    }
}