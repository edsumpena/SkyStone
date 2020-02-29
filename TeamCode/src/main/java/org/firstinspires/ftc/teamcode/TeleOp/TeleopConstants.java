package org.firstinspires.ftc.teamcode.TeleOp;

public class TeleopConstants {
    public static double liftSpeedSlow = 0.3;
    public static double drivePowerNormal = 0.8;
    public static double drivePowerTurbo = 1;
    public static double drivePowerSlow = 0.5;
    public static double turnPower = 0.7;
    public static double intakePower = 1;
    public static double liftPower = 1;
    public static double clawServo1PosClose = 0.075;    //@TODO Get clawServo1 & clawServo2 positions
    public static double clawServo1PosOpen = 0.3003;
    public static double clawServo1PosReceive = 0.54;
    public static double clawServo1Prep = 0.2196;
    public static double clawServo1Capstone = 0.014;
    public static double clawServo2Block = 0.74281;
    public static double clawServo2PosClose = 0.522;
    public static double clawServo2PosOpen = 1;
    public static double clawServo2PosAuto = 1;
    public static double clawServo2CapstoneOld = 0.6;
    public static double clawServo2CapstoneNew = 0.498;// updated per Michael;  0.641;
    public static double transferLockPosUp = 0.4367;
    public static double transferLockPosOut = 0;
    public static double transferLockPosHalfUnlock = 0.3026;
    public static double foundationLockInit = 0.1215;
    public static double foundationLockUnlock = 0.62;
    public static double foundationLockLock = 0.16;
    public static double foundationLockHalfUnlock = 0.31558;


    public static double transferHornPosReady = 1.0;
    public static double transferHornPosPush = 0.327599;
    public static double transferHornCapstone = 0.652;
    public static double clawInitPosReset = 0;
    public static double clawInitPosCapstone = 0.6623;
    public static double clawInitPosCapstoneForReal = 0.547;
    public static double innerTransferPosOpen = 0.411;
    public static double innerTransferPosClosed = 0.749;     //@TODO Get servo position innerTransfer "block" position
    public static double innerTransferPosInit = 0.562;
    public static double intakeInitPosLeft = 0.6131;     //@TODO Get intakeInit servo positions
    public static double intakeInitPosRight = innerTransferPosOpen;
    public static double intakeInitPosReset = 0.3108;

    public static double autoClaw1Retracted = 0.06; // stove away
    public static double autoClaw1Drop = 0.303; // dropping onto foundation
    public static double autoClaw1Stone = 0.76; // store behind robot
    public static double autoClaw1Extended = 0.56; // move to get stone

    public static double autoClaw1Retracted_blue = 0.95;
    public static double autoClaw1Drop_blue = 0.6429;
    public static double autoClaw1Stone_blue = 0.2633;
    public static double autoClaw1Extended_blue = 0.44;

    public static double autoClaw2PickUp = 0.9028;
    public static double autoClaw2Init = 0.749;
    public static double autoClaw2Prep = 0.5844;
    public static double autoClaw2Grabbing = 0.45;

    public static double autoClaw2PickUp_blue = 0.1675;
    public static double autoClaw2Init_blue = 0.1275;
    public static double autoClaw2Prep_blue = 0.3249;
    public static double autoClaw2Grabbing_blue = 0.45;

    public static double autoClaw3Init = 0.7951;
    public static double autoClaw3Closed = 0.5614;
    public static double autoClaw3Open = 0.3258;

    public static double autoClaw3Init_blue = 0.3029;
    public static double autoClaw3Closed_blue = 0.5532;
    public static double autoClaw3Open_blue = 0.7528;

    public static double parkingServoPosUnlock = 0.18;
    public static double parkingServoPosLock = 0.35;

    public static double liftOdometryDown = 0;
    public static double liftOdometryUp = 1;

    public static int[] stoneEncoderValues = new int[] {0, -681, -1120, -1428, -1806};
}
