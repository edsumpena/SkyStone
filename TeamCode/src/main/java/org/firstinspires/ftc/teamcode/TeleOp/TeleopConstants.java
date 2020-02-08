package org.firstinspires.ftc.teamcode.TeleOp;

public class TeleopConstants {

    public static double liftSpeedSlow = 0.3;

    public static double drivePowerNormal = 0.8;
    public static double drivePowerTurbo = 1;
    public static double drivePowerSlow = 0.5;
    public static double turnPower = 0.5;
    public static double intakePower = 0.8;
    public static double liftPower = 1;

    public static double clawServo1PosClose = 0.075;    //@TODO Get clawServo1 & clawServo2 positions
    public static double clawServo1PosOpen = 0.3003;
    public static double clawServo1PosReceive = 0.54;
    public static double clawServo1Prep = 0.2196;
    public static double clawServo1Capstone = 0.307;

    public static double clawServo2Block = 0.74281;
    public static double clawServo2PosClose = 0.522;
    public static double clawServo2PosOpen = 0.95;

    public static double transferLockPosPlatform = 0.465;
    public static double transferLockPosUp = 0.418;
    public static double transferLockPosOut = 0.221;

    public static double foundationLockUnlock = 0.44;//0.304;
    public static double foundationLockLock = 0.148;//0.167;

    public static double transferHornPosReady = 0.62;
    public static double transferHornPosPush = 0;
    public static double transferHornCapstone = 0.0523;

    public static double clawInitPosReset = 0.1;
    public static double clawInitPosCapstone = 0.6623;
    public static double clawInitPosCapstoneForReal = 0.547;

    public static double innerTransferPosOpen = 0.367; //0.1201 closed,
    public static double innerTransferPosClosed = 0.264;     //@TODO Get servo position innerTransfer "block" position

    public static double intakeInitPosLeft = 0.4787;     //@TODO Get intakeInit servo positions
    public static double intakeInitPosRight = 0.4787;
    public static double intakeInitPosReset = 0.28;

    public static double autoClaw1Retracted = 0; // stove away
    public static double autoClaw1Drop = 0.26; // dropping onto foundation
    public static double autoClaw1Stone = 0.7; // store behind robot
    public static double autoClaw1Extended = 0.5; // move to get stone


    public static double autoClaw1Retracted_blue = 0.95;
    public static double autoClaw1Drop_blue = 0.73;
    public static double autoClaw1Stone_blue = 0.23;
    public static double autoClaw1Extended_blue = 0.44;

    public static double autoClaw2PickUp = 0.9028;
    public static double autoClaw2Init = 0.77635;
    public static double autoClaw2Prep = 0.6268;
    public static double autoClaw2Grabbing = 0.45;

    public static double autoClaw2PickUp_blue = 0.38207;
    public static double autoClaw2Init_blue = 0.5221;
    public static double autoClaw2Prep_blue = 0.73;
    public static double autoClaw2Grabbing_blue = 0.7934;

    public static double autoClaw3Init = 0.8554;
    public static double autoClaw3Closed = 0.5709;
    public static double autoClaw3Open = 0.3669;

    public static double autoClaw3Init_blue = 0.071297;
    public static double autoClaw3Closed_blue = 0.3694;
    public static double autoClaw3Open_blue = 0.607;

    public static double parkingServoPosUnlock = 0.4959;
    public static double parkingServoPosLock = 0.3815;

    public static double liftOdometryDown = 0.142;
    public static double liftOdometryUp = 0.425;

    public static int[] stoneEncoderValues = new int[] {0, -681, -1120, -1428, -1806};
}
