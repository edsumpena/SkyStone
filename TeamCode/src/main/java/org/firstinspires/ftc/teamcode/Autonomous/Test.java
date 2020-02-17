package org.firstinspires.ftc.teamcode.Autonomous;

import android.content.Intent;
import android.support.annotation.NonNull;
import android.util.Log;
import android.widget.Toast;

import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.Autonomous.Vision.Align;
import org.firstinspires.ftc.teamcode.Experimental.PathfinderApp.RunTrajectoryFromString;
import org.firstinspires.ftc.teamcode.Experimental.PathfinderApp.TrajectoryStringConverter;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

@Autonomous(name = "TestAlign", group = "LinearOpMode")
@Disabled
public class Test extends LinearOpMode {
    private Align align;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private FourWheelMecanumDrivetrain drivetrain;
    private String trajectory = null;
    private HardwareMap hwMap;
    private static final String TFOD_MODEL_ASSET = "skystoneTFOD_v2_[105-15].tflite";
    private static final String LABEL_FIRST_ELEMENT = "skystone";
    private static final String LABEL_SECOND_ELEMENT = "stone";
    private static int imgWidth = 0;
    private static final String VUFORIA_KEY = "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";

    @Override
    public void runOpMode(){
        HardwareMap hwMap = new HardwareMap(hardwareMap);
        drivetrain = new FourWheelMecanumDrivetrain(hwMap);
        align = new Align(hwMap, this, DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.resetEncoders();
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);
        String trajectory = loadTrajectoryFromDatabase() != null ? loadTrajectoryFromDatabase() : DriveConstant.trajectoryString;

        RunTrajectoryFromString trajRunner = new RunTrajectoryFromString(drive, trajectory);

        drivetrain.resetEncoders();

        waitForStart();

        trajRunner.runTrajectory();
    }

    public String loadTrajectoryFromDatabase(){
        trajectory = null;
        DatabaseReference rootRef = FirebaseDatabase.getInstance().getReference().child("Trajectories");
        rootRef.addListenerForSingleValueEvent(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                trajectory = rootRef.child("Red_Quarry-[1,4]").getKey();
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {
                Log.d("ERROR", "Failed to get missingSwitch state.");
            }
        });
        return trajectory;
    }
}
