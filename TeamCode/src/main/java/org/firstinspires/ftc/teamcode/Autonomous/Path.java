package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Align;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;
import org.firstinspires.ftc.teamcode.PID.RobotLogger;
import org.firstinspires.ftc.teamcode.PID.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.PID.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.PID.DriveConstantsPID.rear_ratio;

public class Path {
    private Pose2d startingPos;
    private SampleMecanumDriveBase straightDrive;
    private SampleMecanumDriveBase strafeDrive;
    private SampleMecanumDriveBase _drive;
    private int step_count = 0;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    private Align align;
    private HardwareMap hwMap;
    private LinearOpMode opMode;
    private List<Recognition> tfod;
    private com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    private static String TAG = "AutonomousPath";
    private Pose2d currentPos;
    private BNO055IMU imu;
    private Telemetry telemetry;
    private String path_file;
    private int first_skystone_location = 0;

    public Path(HardwareMap hwMap, LinearOpMode opMode, SampleMecanumDriveBase straightDrive,
                com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, BNO055IMU imu, Telemetry telemetry) {
        this.straightDrive = straightDrive;
        this.strafeDrive = straightDrive;
        //this.startingPos = startingPos;
        this.hwMap = hwMap;
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        align = new Align(hwMap, opMode, DcMotor.ZeroPowerBehavior.BRAKE);


        _drive = straightDrive;
        this.imu = imu;
        this.telemetry = telemetry;
        //vu = new VuforiaCamLocalizer(hardwareMap);
    }
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
            RobotLogger.dd(TAG, "x < y, strafe first and then line");
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
    /*
    input: last pose from previous move;
    return: drive instance;
     */
    private SampleMecanumDriveBase DriveBuilderReset(boolean isStrafe, boolean init_imu, String label) {
        currentPos = _drive.getPoseEstimate();
        Pose2d newPos = currentPos;
        Pose2d error_pose = _drive.follower.getLastError();
        RobotLog.dd(TAG, "start new step: %s, count[%d], currentPos %s, errorPos %s",
                label, step_count, currentPos.toString(), error_pose.toString());
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS == false){
            sleep_millisec((int) DriveConstantsPID.TEST_PAUSE_TIME);
        }
        /*
        if (DriveConstantsPID.drvCorrection)
        {
            boolean done = false;
            if ((abs(error_pose.getX())>1.5))// && (abs(error_pose.getX())>abs(error_pose.getY())))
            {
                RobotLogger.dd(TAG, "pose correction by straight move");
                _drive.resetFollowerWithParameters(false, false);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .setReversed((error_pose.getX()>0)?false:true)
                                .lineTo(new Vector2d(newPos.getX() + error_pose.getX(), newPos.getY()))
                                .build());
                done = true;
                newPos = _drive.getPoseEstimate();
                RobotLogger.dd(TAG, "after pose correction: currentPos %s, errorPos %s",
                        newPos.toString(), _drive.follower.getLastError().toString());
            }
            if ((abs(error_pose.getY())>1.5))// && (abs(error_pose.getX())<abs(error_pose.getY())))
            {
                RobotLogger.dd(TAG, "pose correction by strafing");
                _drive.resetFollowerWithParameters(true, false);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .setReversed(false)
                                .strafeTo(new Vector2d(newPos.getX(), newPos.getY() + error_pose.getY()))
                                .build());
                done = true;
                newPos = _drive.getPoseEstimate();
                RobotLogger.dd(TAG, "after pose correction: currentPos %s, errorPos %s",
                        newPos.toString(), _drive.follower.getLastError().toString());
            }

            if (Math.toDegrees(error_pose.getHeading())>10)
            {
                RobotLog.dd(TAG, "correct heading by turning");
                _drive.resetFollowerWithParameters(false, false);
                _drive.turnSync(error_pose.getHeading());
                done = true;
                newPos = _drive.getPoseEstimate();
            }
            if (done) {
                currentPos = newPos;
            }
        }
         */
        //RobotLogger.dd(TAG, "vuforia localization info: %s", vu.getPoseEstimate().toString());

        if (DriveConstantsPID.RECREATE_DRIVE_AND_BUILDER) {
            if (DriveConstantsPID.USING_BULK_READ)
                _drive = new SampleMecanumDriveREVOptimized(hardwareMap, isStrafe);
            else
                _drive = new SampleMecanumDriveREV(hardwareMap, isStrafe);
        }
        else
            _drive.resetFollowerWithParameters(isStrafe, false);

        //_drive = new SampleMecanumDriveREV(hardwareMap, isStrafe, init_imu);
        _drive.getLocalizer().setPoseEstimate(currentPos);
        _drive.getLocalizer().update();
        if (!isStrafe) {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        } else {
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.STRAFE_BASE_CONSTRAINTS);
        }
        RobotLog.dd(TAG, "drive and builder created, initialized with pose: " + _drive.getPoseEstimate().toString());
        return _drive;
    }
    private void sleep_millisec(int c)
    {
        try {
            Thread.sleep(c);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    private int FollowPathFromXMLFile(Pose2d coordinates[], VuforiaCamLocalizer vLocal, boolean isRed) {
        Pose2d error_pose;
        int xy_len = coordinates.length;
        if (xy_len == 0)
        {
            telemetry.addData("read path XY failure: ", path_file);
            telemetry.update();
            RobotLogger.dd(TAG, "failed to read xml file");
            return -1;
        }
        RobotLogger.dd(TAG, "finished reading path coordinates num: " + Integer.toString(xy_len));
        step_count = 0;

        startingPos = coordinates[step_count];
        RobotLogger.dd(TAG, "step" + Integer.toString(step_count) + coordinates[step_count].toString());
        step_count ++;

        _drive.setPoseEstimate(startingPos);
        //_drive.getLocalizer().setPoseEstimate(startingPos);
        _drive.update();

        double theta;

        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            transferReset();
            initIntakeClaw();
            init();
            prepGrab(true);    //*******

        }
        // step 1;

        DriveBuilderReset(true, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after prepare, start");
        if (first_skystone_location != 1) {
            builder = builder
                    .setReversed(false).strafeTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
            trajectory = builder.build();   //x - 2.812, y + 7.984
            _drive.followTrajectorySync(trajectory);
        }
        else {
            StrafeDiagonalHelper(_drive, new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
        }
        step_count ++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before grab stone! Vuforia local info: " + t.toString());
        }
        RobotLog.dd(TAG, "step1.5, after strafe, to grab");
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            if (isRed)
                grabStone(FieldPosition.RED_QUARY);   //*******
            else
                grabStone(FieldPosition.BLUE_QUARY);
        }

        // step 2;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after grab , to go straight");

        if (DriveConstantsPID.drvCorrection) {
            coordinates[step_count] = new Pose2d(coordinates[step_count].getX() + error_pose.getX(),
                    coordinates[step_count].getY() + error_pose.getY(), coordinates[step_count].getHeading());
            RobotLogger.dd(TAG, "next step after correction: " + coordinates[step_count].toString());
        }

        builder = builder
                .setReversed(false).lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before drop stone! Vuforia local info: " + t.toString());
        }
        RobotLog.dd(TAG, "step2.5, after straight");
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            if (isRed)
                dropStone(FieldPosition.RED_QUARY); //*******
            else
                dropStone(FieldPosition.BLUE_QUARY);
        }


        // step 3;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop 1st stone, to straight move back");

        if (DriveConstantsPID.drvCorrection) {
            coordinates[step_count] = new Pose2d(coordinates[step_count].getX() + error_pose.getX(),
                    coordinates[step_count].getY() + error_pose.getY(), coordinates[step_count].getHeading());
            RobotLogger.dd(TAG, "next step after correction: " + coordinates[step_count].toString());
        }

        builder = builder
                .setReversed(true).lineTo((new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY())));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;
        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before grab 2nd stone! Vuforia local info: " + t.toString());
        }
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            prepGrab(false); //*******
        }

        sleep_millisec(100);


        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            if (isRed)
                grabStone(FieldPosition.RED_QUARY);   //*******
            else
                grabStone(FieldPosition.BLUE_QUARY);
        }

        // step 4;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                "after straight move, grabbed 2nd, to straight move");
        if (DriveConstantsPID.drvCorrection) {
            coordinates[step_count] = new Pose2d(coordinates[step_count].getX() + error_pose.getX(),
                    coordinates[step_count].getY() + error_pose.getY(), coordinates[step_count].getHeading());
            RobotLogger.dd(TAG, "next step after correction: " + coordinates[step_count].toString());
        }
        builder = builder
                .setReversed(false).lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before drop 2nd stone! Vuforia local info: " + t.toString());
        }
        RobotLog.dd(TAG, "step4.5, after straight move, to drop");
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            if (isRed)
                dropStone(FieldPosition.RED_QUARY);   //*******
            else
                dropStone(FieldPosition.BLUE_QUARY);
        }

        // step 5
        DriveBuilderReset(true, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop 2nd stone, to strafe");
        builder = builder
                .setReversed(false).strafeTo(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        // step 6
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop and strafe");
        theta = _drive.getExternalHeading() >= 0 ? _drive.getExternalHeading() :
                _drive.getExternalHeading() + 2 * PI;

        if (theta > PI)
            _drive.turnSync(-(_drive.getExternalHeading() - 3 * PI / 2) + PI / 6);
        else
            _drive.turnSync(-(_drive.getExternalHeading() + 2 * PI - 3 * PI / 2) + PI / 6);

        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
            hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
        }

        sleep_millisec(100);
        // step 6
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after foundation unlock, to straight move closer to foundation");
        builder = builder.setReversed(true).lineTo(new Vector2d(_drive.getPoseEstimate().getX(),
                _drive.getPoseEstimate().getY() + coordinates[step_count].getY()));

        //builder = builder.setReversed(true).lineTo(new Vector2d(coordinates[step_count].getX(),
        //        coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        // step 7
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
            hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
        }
        sleep_millisec(400);
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop fundation,, to spline ");
        builder.setReversed(false)
                .splineTo(new Pose2d(new Vector2d(_drive.getPoseEstimate().getX() - coordinates[step_count].getX(),
                        _drive.getPoseEstimate().getY() - coordinates[step_count].getY()), coordinates[step_count].getHeading()));
        /*
        builder = builder.setReversed(false)
                .splineTo(new Pose2d(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()), coordinates[step_count].getHeading()));
         */
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        // step 8
        if (DriveConstantsPID.ENABLE_ARM_ACTIONS) {
            hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
            hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
        }

        sleep_millisec(100);

        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", spline, back to parking");
        //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder
                .setReversed(false).splineTo(new Pose2d(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()), coordinates[step_count].getHeading()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectorySync(trajectory);
        step_count ++;

        while(opMode.opModeIsActive()){
            String o = "";
            for(Pose2d p2d : coordinates)
                o += p2d.toString() + ", ";
            telemetry.addData("Pose2d", o);
            telemetry.update();
        }
        return 0;
    }
    public void RedQuary(int[] skystonePositions, VuforiaCamLocalizer vuLocalizer) {
        String tmp = "path_red_.xml";
        first_skystone_location = skystonePositions[0];
        path_file = tmp.substring(0, 8) + Integer.toString(skystonePositions[0])
                + tmp.substring(9);
        RobotLogger.dd(TAG, "to read XY coordinates from " + path_file);

        Pose2d xys1[] = DriveConstantsPID.parsePathXY(path_file);
        FollowPathFromXMLFile(xys1, vuLocalizer, true);
    }

    public void RedFoundationPark() {
        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset();
        initIntakeClaw();
        sleep_millisec(5000);


        intake(0);

        sleep_millisec(15000);


        hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
        hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

        sleep_millisec(10000);


        /*try {
            Thread.sleep(18000);
        } catch (Exception e) {
        }
        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeRight(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);*/
    }

    public void BlueQuary(int[] skystonePositions, VuforiaCamLocalizer vuLocalizer) {    // (-x, y)
        String tmp = "path_blue_.xml";
        first_skystone_location = skystonePositions[0];
        path_file = tmp.substring(0, 9) + Integer.toString(skystonePositions[0])
                + tmp.substring(10);
        RobotLogger.dd(TAG, "to read XY coordinates from " + path_file);

        Pose2d xys1[] = DriveConstantsPID.parsePathXY(path_file);
        FollowPathFromXMLFile(xys1, vuLocalizer, false);
    }

    public void BlueFoundationPark() {
        //hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset();
        initIntakeClaw();
        sleep_millisec(5000);

        intake(0);
        sleep_millisec(15000);


        hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
        hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);

        sleep_millisec(10000);

        /*try {
            Thread.sleep(18000);
        } catch (Exception e) {
        }
        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.strafeLeft(6).setReversed(false).forward(28);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);
        intake(0);*/
    }

    public void BlueFoundationDrag() {
        transferReset();
        initIntakeClaw();
        startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));

        sleep_millisec(5000);

        straightDrive.getLocalizer().setPoseEstimate(startingPos);
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.lineTo(new Vector2d(20.736, 63.936)).lineTo(new Vector2d(20.736, 48.936))
                .strafeTo(new Vector2d(68.144, 48.936));
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

        intake(0);
        align.setPower(0.13, 0.25);
        align.foundation(FieldPosition.BLUE_QUARY);
        transferReset();

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), straightDrive.getExternalHeading()));
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                .setReversed(true).lineTo(new Vector2d(70.0, 58.72)).setReversed(false);
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
        straightDrive.getLocalizer().update();
        builder = new TrajectoryBuilder(straightDrive.getPoseEstimate(), DriveConstantsPID.BASE_CONSTRAINTS);
        builder = builder.setReversed(false).lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 60.44))
                .lineTo(new Vector2d(17.552, 60.44));
        trajectory = builder.build();
        straightDrive.followTrajectorySync(trajectory);
    }

    public void updateTFODData(List<Recognition> tfod) {
        this.tfod = tfod;
        updateTFOD();
    }

    public Pose2d getPoseEstimate() {
        return straightDrive.getLocalizer().getPoseEstimate();
    }

    public double getExternalHeading() {
        return straightDrive.getExternalHeading();
    }

    private void updateTFOD() {
        align.updateTFOD(tfod);
    }

    public void updateHeading() {
        align.updateExternalHeading(Math.toDegrees(straightDrive.getExternalHeading()));
    }

    // TODO : servo updates are non-blocking, this is unnecessary
    private void transferReset() {
        Thread thread = new Thread() {
            public void run() {
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosBlock);
            }
        };
        thread.start();
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

    private void prepGrab(boolean starting) {
        if(starting) {
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
            sleep_millisec(200);

            hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
            sleep_millisec(200);

            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
            sleep_millisec(200);
        } else {
            hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
            sleep_millisec(200);

            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
            sleep_millisec(200);

            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
            sleep_millisec(200);
        }

    }

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

    private void dropStone(FieldPosition fieldPosition) {
        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop);
        sleep_millisec(400);

        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
        sleep_millisec(200);

        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
        sleep_millisec(200);

        hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);
        sleep_millisec(400);

        hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);
        sleep_millisec(200);

        hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
        sleep_millisec(200);

    }

    private void intake(double power) {
        Thread thread = new Thread() {
            public void run() {
                hwMap.leftIntake.setPower(-power);
                hwMap.rightIntake.setPower(power);
            }
        };
        thread.start();
    }

    private void driveTime(double speed, int milisecs) {
        hwMap.frontRight.setPower(speed);
        hwMap.frontLeft.setPower(speed);
        hwMap.backRight.setPower(speed);
        hwMap.backLeft.setPower(speed);

        try {
            Thread.sleep(milisecs);
        } catch (Exception e) {
        }

        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    private void updatePoseFromStrafe(double yOdoInitPos, boolean left) {
        double strafeDistance = Math.abs(hwMap.rightIntake.getCurrentPosition() - yOdoInitPos) /
                DriveConstantsPID.odoEncoderTicksPerRev * StandardTrackingWheelLocalizer.GEAR_RATIO *
                2 * Math.PI * StandardTrackingWheelLocalizer.WHEEL_RADIUS;

        RobotLog.dd("Current Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
        RobotLog.dd("Strafe Distance", String.valueOf(strafeDistance));
        RobotLog.dd("Direction", left ? "Left" : "Right");
        if (!left)
            strafeDistance = -strafeDistance;

        double heading = straightDrive.getExternalHeading() < 0 ? Math.PI * 2 - straightDrive.getExternalHeading() : straightDrive.getExternalHeading();

        straightDrive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(straightDrive.getPoseEstimate().getX(),
                straightDrive.getPoseEstimate().getY() + strafeDistance), heading));
        straightDrive.getLocalizer().update();
        RobotLog.dd("Updated Position (X, Y, Heading)", straightDrive.getPoseEstimate().toString());
    }

    public void odometryStrafe(double power, double inches, boolean right) {
        double counts = inches / (2 * PI * 1.25) * 1550.0;
        int sidewaysStart = hwMap.rightIntake.getCurrentPosition();

        if (!right)
            power = -power;

        hwMap.frontRight.setPower(-power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power * rear_ratio);
        hwMap.backLeft.setPower(-power * rear_ratio);

        while (opMode.opModeIsActive()) {
            int sideways = hwMap.rightIntake.getCurrentPosition();

            int sidewaysDiff = Math.abs(sideways - sidewaysStart);

            if (opMode != null) {
                opMode.telemetry.addData("Side", sidewaysDiff);
                opMode.telemetry.addData("Target", counts);
                opMode.telemetry.update();
            }

            if (sidewaysDiff >= counts) {
                break;
            }
        }
        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    private void init() {
        Thread thread = new Thread() {
            public void run() {
                //hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);
            }
        };
        thread.start();
        sleep_millisec(50);

    }
}