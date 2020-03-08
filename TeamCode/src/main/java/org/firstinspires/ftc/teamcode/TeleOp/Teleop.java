package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.All.Lift;
import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.GamepadButtons;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.OnOffButton;

import java.util.ArrayList;


/*
 * -Gamepad A:
 *  -Start + Back to switch modes
 *  -Right Joystick controls Right Wheel
 *  -Left Bumper strafes left
 *  -Right Bumper strafes right
 *  -Y & B controls servo1
 *  -X & A controls servo2
 * -Gamepad B:
 *  -Y and A to intake and outake
 *  -Left joystick and Right joysticks to control the lift
 *  -Adjust the HardwareMap configurations in HardwareMap.java class
 *  -Adjust Intake/Lift power in TeleopConstants.java class
 */

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private boolean intake = false;
    private boolean outake = false;
    private final double normalSpeed = TeleopConstants.drivePowerNormal;
    private final double turboSpeed = TeleopConstants.drivePowerTurbo;
    private final double slowSpeed = TeleopConstants.drivePowerSlow;
    private double turnSpeed = TeleopConstants.turnPower;
    private boolean runLogic = false;
    private boolean switchBlocker = false;
    private boolean tobyMode = false;
    private HardwareMap hwMap;
    private boolean manualOverride = false;
    private boolean blockerCapstone = false;
    private boolean transHornFlag = false;

    private ArrayList<OnOffButton> buttonLogic = new ArrayList<>();

    private boolean blocker = false;

    private boolean dummy = false;
    private boolean transferHornDummy = false;
    private boolean parking = false;
    private ArrayList<String> kVData = new ArrayList<>();

    private FourWheelMecanumDrivetrain drivetrain;
    private Lift lift;

    private int autoClawStage = 0;
    private boolean run = true;
    private boolean isRed = true;
    private boolean blockerRight = false;
    private boolean blockerLeft = false;

    private long lastUpdateTime;

    private boolean initBlocker = false;
    private boolean buttonBlocker = false;

    public void runOpMode() {
        hwMap = new HardwareMap(hardwareMap);

        runLogic = true;

        drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

        drivetrain.setSpeedMultiplier(normalSpeed);
        drivetrain.resetEncoders();

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = new Lift(hardwareMap);

        lift.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setSpeedMultiplier(TeleopConstants.liftPower);

        lift.resetEncoders();

        //lift.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.liftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwMap.liftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buttonLogic.add(new OnOffButton(gamepad2, gamepad2, GamepadButtons.A, GamepadButtons.B, //Intake-A & B
                new DcMotor[]{hwMap.leftIntake, hwMap.rightIntake},
                new double[][]{{-TeleopConstants.intakePower, 0}, {TeleopConstants.intakePower, 0}},
                new double[]{0.5, -0.5}));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.Y, new Servo[]{hwMap.foundationLock, hwMap.transferLock},   //Foundation Lock-Y
                new double[][]{{TeleopConstants.foundationLockLock, TeleopConstants.foundationLockUnlock},
                        {TeleopConstants.transferLockPosUp, TeleopConstants.transferLockPosOut}}));
        /*buttonLogic.add(new OnOffButton(gamepad2, gamepad2, GamepadButtons.LEFT_BUMPER, GamepadButtons.RIGHT_BUMPER, //Intake-A & B
                new Servo[]{hwMap.clawServo1},
                new double[][]{{TeleopConstants.clawServo1PosOpen, TeleopConstants.clawServo1PosClose}},
                new double[]{TeleopConstants.clawServo1Block}));*/
        //buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.X,
        //        new Servo[] {hwMap.parkingServo},
        //        new double[][]{ {TeleopConstants.parkingServoPosUnlock, TeleopConstants.parkingServoPosLock} }));
        /*buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.DPAD_DOWN,
                new Servo[]{hwMap.innerTransfer},
                new double[][]{{TeleopConstants.innerTransferPosBlock, TeleopConstants.innerTransferPosTucked}}));*/
        /*buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.LEFT_TRIGGER, new Servo[]{hwMap.transferHorn},
                new double[][]{{TeleopConstants.transferHornPosPush, TeleopConstants.transferHornPosReady}}));*/
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.DPAD_LEFT, new Servo[]{hwMap.liftOdometry},
                new double[][]{{TeleopConstants.liftOdometryDown, TeleopConstants.liftOdometryUp}}));

        telemetry.addData("Status", "Ready");
        hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);

        //hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1TeleOp);
        //hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Stowed);
        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);

        waitForStart();

        driveLoop();
        liftLoop();
        toggleLoop();
        parkingLoop(this);
        armDelayHandler(this);
        initIntakeClawArm(this);
        autoClawThread();

        String val = DriveConstant.getString(AppUtil.ROOT_FOLDER + "/FIRST/PrevRunPath.txt");
        if (val.equalsIgnoreCase(FieldPosition.RED_QUARY.toString()) || val.equalsIgnoreCase(FieldPosition.RED_FOUNDATION_PARK.toString())) {
            isRed = true;
            hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
            Path.sleep_millisec_opmode(200, this);
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);
            Path.sleep_millisec_opmode(200, this);
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted);
            Path.sleep_millisec_opmode(200, this);
        } else if (val.equalsIgnoreCase(FieldPosition.BLUE_QUARY.toString()) || val.equalsIgnoreCase(FieldPosition.BLUE_FOUNDATION_PARK.toString())) {
            isRed = false;
            hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init_blue);
            Path.sleep_millisec_opmode(200, this);
            hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init_blue);
            Path.sleep_millisec_opmode(200, this);
            hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted_blue);
            Path.sleep_millisec_opmode(200, this);
        }
        hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosClosed);

        while (opModeIsActive()) {

            if (gamepad2.a && buttonLogic.get(0).getState()[0]) {
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                Path.sleep_millisec_opmode(200, this);
                /*hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosReceive);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);
                try{
                    Thread.sleep(200);
                } catch (Exception e){}*/
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);
                Path.sleep_millisec_opmode(200, this);
            }

            //------------------------------===Capstone===------------------------------------------

            if (gamepad2.dpad_up && !blockerCapstone) {
                blockerCapstone = true;
                telemetry.addData("here1", "f");
                telemetry.update();

                //James's Version:

                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Capstone);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2CapstoneOld);
                Path.sleep_millisec_opmode(200, this);

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);

                //Old Version:

                /*hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Capstone);
                hwMap.transferHorn.setPosition(TeleopConstants.transferHornCapstone);

                try{
                    Thread.sleep(750);
                } catch(Exception e){}

                /*if (!buttonLogic.get(2).getState()[0])
                    buttonLogic.get(2).manualActivate(true, false);

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosClosed);

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);

                try{
                    Thread.sleep(300);
                } catch (Exception e){}

                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosReceive);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);

                try{
                    Thread.sleep(500);
                } catch (Exception e){}

                hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);

                try{
                    Thread.sleep(1000);
                } catch (Exception e){}*/
            } else if (!gamepad2.dpad_up && blockerCapstone) {
                telemetry.addData("here2", "f");
                telemetry.update();
                blockerCapstone = false;

                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Capstone);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2CapstoneOld);
                Path.sleep_millisec_opmode(200, this);

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);
                /*hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosReceive);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);
                try{
                    Thread.sleep(200);
                } catch(Exception e){}*/
            }

            if(gamepad1.dpad_down && !blocker){
                blocker = true;
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                Path.sleep_millisec_opmode(200, this);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
            }

            if (gamepad2.dpad_down) {
                hwMap.clawServo1.setPosition(0.36);
                try {
                    Thread.sleep(200);
                } catch (Exception e) {
                }
            }

            if (gamepad2.right_trigger >= 0.5) {
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2CapstoneNew);
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
            }

            if (gamepad1.a) {
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Capstone);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2CapstoneOld);
            }

            //------------------------------===Driving/Strafing===------------------------------------------

            if (gamepad1.right_bumper && !switchBlocker) {
                switchBlocker = true;
                tobyMode = !tobyMode;
            } else if (!gamepad1.right_bumper && switchBlocker) {
                switchBlocker = false;
            }

            if (gamepad2.right_stick_y != 0 || gamepad2.left_stick_y != 0) {    //??? Ask Mechanical
                lift.moveLift(gamepad2.right_stick_y + gamepad2.left_stick_y * TeleopConstants.liftSpeedSlow);
            } else {
                lift.stop();
            }

            //--------------------------------------------===AutoClaw===-------------------------------------------------

            if (gamepad1.right_bumper && !blockerRight) {
                blockerRight = true;
                if (autoClawStage + 1 <= 4)
                    autoClawStage += 1;
                else
                    autoClawStage = 0;
                run = true;
            } else if (!gamepad1.right_bumper && blockerRight)
                blockerRight = false;

            if (gamepad1.left_bumper && !blockerLeft) {
                blockerLeft = true;
                autoClawStage = 0;
                run = true;
            } else if (!gamepad1.left_bumper && blockerLeft)
                blockerLeft = false;

            telemetry.addData("LeftForwardOdometry", hwMap.leftIntake.getCurrentPosition());
            telemetry.addData("RightForwardOdometry", hwMap.liftTwo.getCurrentPosition());
            telemetry.addData("SidewaysOdometry", hwMap.rightIntake.getCurrentPosition());

            telemetry.addData("FrontLeft", hwMap.frontLeft.getCurrentPosition());
            telemetry.addData("BackLeft", hwMap.backLeft.getCurrentPosition());
            telemetry.addData("FrontRight", hwMap.frontRight.getCurrentPosition());
            telemetry.addData("BackRight", hwMap.backRight.getCurrentPosition());

            telemetry.addData("LiftOne", hwMap.liftOne.getCurrentPosition());

            telemetry.update();

        }


    }

    private void detectBlock(LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread thread = new Thread() {
            public void run() {
                if (!hwMap.intakeDetect.getState() && !blocker && !manualOverride) {
                    blocker = true;
                    hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                    Path.sleep_millisec_opmode(500, opmode);
                    //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosTucked);
                    Path.sleep_millisec_opmode(500, opmode);
                    hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                    //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosExtended);
                    Path.sleep_millisec_opmode(300, opmode);
                    blocker = false;
                }
            }
        };

        Thread interruptDetect = new Thread() {
            public void run() {
                while (blocker) {
                    if (manualOverride)
                        thread.interrupt();
                }
            }
        };
        thread.start();
        Path.sleep_millisec_opmode(100, opmode);
        interruptDetect.start();
    }

    private void SAMPLE_TWO_BUTTON_LOGIC(HardwareMap hwMap) {
        if (gamepad2.y && !blocker) {
            if (!intake) {
                intake = true;
                outake = false;
            } else {
                intake = false;
                outake = false;
            }
            blocker = true;
        }

        if (gamepad2.a && !blocker) {
            if (!outake) {
                intake = false;
                outake = true;
            } else {
                intake = false;
                outake = false;
            }
            blocker = true;
        }

        if (!gamepad2.y && !gamepad2.a) {
            blocker = false;
        }

        if (intake) {
            hwMap.rightIntake.setPower(-TeleopConstants.intakePower);
            hwMap.leftIntake.setPower(TeleopConstants.intakePower);
        }

        if (outake) {
            hwMap.rightIntake.setPower(TeleopConstants.intakePower);
            hwMap.leftIntake.setPower(-TeleopConstants.intakePower);
        }

        if (!intake && !outake) {
            hwMap.rightIntake.setPower(0);
            hwMap.leftIntake.setPower(0);
        }
    }

    private void driveLoop() {
        Thread drive = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    // telemetry.addData("Info", "Press START + BACK on GAMEPAD1 to switch drive modes!");

                    if (tobyMode) {
                        drivetrain.setSpeedMultiplier(turboSpeed);
                        turnSpeed = 1;
                    } else {
                        drivetrain.setSpeedMultiplier(normalSpeed);
                        turnSpeed = TeleopConstants.turnPower;
                    }
                    // telemetry.addData("Current Drive Mode", "CLASSIC MODE");
                    drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

                    double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

                    // TODO: change to different button for jimmy

                    if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {

                        double speed;

                        if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                            speed = 0;
                        } else if (gamepad1.right_stick_y == 0) {
                            speed = Math.sqrt(2) * Math.abs(gamepad1.left_stick_x);
                        } else if (gamepad1.left_stick_x == 0) {
                            speed = Math.abs(gamepad1.right_stick_y);
                        } else {
                            speed = Math.min(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y), 1);
                        }

                        double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                        drivetrain.MoveAngle(speed, angle, turn);
                    } else {
                        drivetrain.stop();
                    }

                    /*telemetry.addData("Gamepad 1 State", gamepad1.toString());

                    long deltaTime = System.nanoTime() - lastUpdateTime;
                    telemetry.addData("Time since last update", deltaTime);
                    if (deltaTime != 0) {
                        telemetry.addData("UPS", 1000000000 / deltaTime);
                    }

                    lastUpdateTime = System.nanoTime();*/

                }
            }
        };
        drive.start();
    }

    private void liftLoop() {
        Thread drive = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad2.right_stick_y != 0) {
                        lift.moveLift(gamepad2.right_stick_y);
                    } else if (gamepad2.left_stick_y != 0) {
                        lift.moveLift(gamepad2.left_stick_y * 0.5);
                    } else {
                        lift.stop();
                    }
                    // lift.detectResetEncoder();
                }
            }
        };
        drive.start();
    }

    private void toggleLoop() {
        Thread toggle = new Thread() {
            public void run() {
                while (opModeIsActive())
                    for (OnOffButton onOffButton : buttonLogic)
                        onOffButton.getGamepadStateAndRun();
            }
        };
        toggle.start();
    }

    private void armDelayHandler(LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread armDelay = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad2.left_bumper && !blocker) {
                        blocker = true;

                        if (!intake) {
                            intake = true;
                            outake = false;
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                        } else {
                            intake = false;
                            outake = false;
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Prep);
                            Path.sleep_millisec_opmode(250, opmode);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                            Path.sleep_millisec_opmode(250, opmode);
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                        }
                    }

                    if (gamepad2.right_bumper && !blocker) {
                        blocker = true;
                        if (!outake) {
                            intake = false;
                            outake = true;
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosReceive);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);
                        } else {
                            intake = false;
                            outake = false;
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Prep);
                            Path.sleep_millisec_opmode(250, opmode);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                            Path.sleep_millisec_opmode(250, opmode);
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                        }
                    }

                    if (!gamepad2.right_bumper && !gamepad2.left_bumper && !gamepad1.dpad_down)
                        blocker = false;
                }
            }
        };
        armDelay.start();
    }

    private void parkingLoop(LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread parkingServ = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad2.left_trigger >= 0.5) {
                        if (gamepad2.x && !dummy) {
                            if (!parking) {
                                //hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Up - 0.08);
                                Path.sleep_millisec_opmode(300, opmode);
                                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosUnlock);
                                parking = true;
                            } else {
                                hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                                Path.sleep_millisec_opmode(300, opmode);
                                //hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1TeleOp);
                                parking = false;
                            }
                            dummy = true;
                        } else if (!gamepad2.x && dummy) {
                            dummy = false;
                        }
                    }

                    if (gamepad2.left_trigger >= 0.5 && !transferHornDummy) {
                        if (!transHornFlag) {
                            hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                            transHornFlag = true;
                        } else {
                            hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);
                            transHornFlag = false;
                        }
                        transferHornDummy = true;
                    } else if (gamepad2.left_trigger < 0.5 && transferHornDummy) {
                        transferHornDummy = false;
                    }
                }
            }
        };
        parkingServ.start();
    }

    private void initIntakeClawArm(LinearOpMode mode) { //TODO Replace Thread.sleep() with sleep_milisecond_opmode
        LinearOpMode opmode = mode;
        Thread t = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad2.dpad_right && !initBlocker && !buttonBlocker) {
                        initBlocker = true;
                        buttonBlocker = true;

                        hwMap.transferLock.setPosition(TeleopConstants.transferLockPosHalfUnlock);

                        try {
                            Thread.sleep(300);
                        } catch (Exception e) {
                        }

                        hwMap.foundationLock.setPosition(TeleopConstants.foundationLockHalfUnlock);

                        try {
                            Thread.sleep(300);
                        } catch (Exception e) {
                        }

                        hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);

                        try {
                            Thread.sleep(600);
                        } catch (Exception e) {
                        }

                        hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                        try {
                            Thread.sleep(600);
                        } catch (Exception e) {
                        }

                        //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                        hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);

                        try {
                            Thread.sleep(1000);
                        } catch (Exception e) {
                        }

                        hwMap.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);

                        try {
                            Thread.sleep(1000);
                        } catch (Exception e) {
                        }

                        hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                        try {
                            Thread.sleep(300);
                        } catch (Exception e) {
                        }

                        hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosPush);

                        try {
                            Thread.sleep(500);
                        } catch (Exception e) {
                        }

                        hwMap.transferHorn.setPosition(TeleopConstants.transferHornPosReady);

                        try {
                            Thread.sleep(500);
                        } catch (Exception e) {
                        }

                        initBlocker = false;
                    }

                    if(!gamepad2.dpad_right && buttonBlocker)
                        buttonBlocker = false;
                }
            }
        };

        t.start();
    }

    private void autoClawThread() {  //TODO Replace Thread.sleep() with sleep_milisecond_opmode
        Thread t = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    switch (autoClawStage) {
                        case 0:
                            if (run) {
                                run = false;
                                if (isRed)
                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
                                else
                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init_blue);

                                try {
                                    Thread.sleep(300);
                                } catch (Exception e) {
                                }

                                if (isRed)
                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);
                                else
                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init_blue);

                                try {
                                    Thread.sleep(300);
                                } catch (Exception e) {
                                }

                                if (isRed)
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted);
                                else
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted_blue);

                                try {
                                    Thread.sleep(300);
                                } catch (Exception e) {
                                }
                            }
                            break;
                        case 1:
                            if (run) {
                                run = false;
                                if (isRed) {
                                    hwMap.redAutoClawJoint2.setPosition(0.85);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                } else {
                                    hwMap.redAutoClawJoint2.setPosition(0.04);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                }
                            }
                            break;
                        case 2:
                            if (run) {
                                run = false;
                                if (isRed) {
                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                } else {
                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                }
                            }
                            break;
                        case 3:
                            if (run) {
                                run = false;
                                if (isRed) {
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                } else {
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                }
                            }
                            break;
                        case 4:
                            if (run) {
                                run = false;
                                if (isRed) {
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop);

                                    try {
                                        Thread.sleep(600);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                } else {
                                    hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);

                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }

                                    hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp_blue);
                                    try {
                                        Thread.sleep(300);
                                    } catch (Exception e) {
                                    }
                                }
                            }
                            break;
                    }
                }
            }
        };
        t.start();
    }

    private void saveDataLoop(HardwareMap hw, LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread loop = new Thread() {
            public void run() {
                long initTime = System.currentTimeMillis();
                while (runLogic) {
                    kVData.add(hw.frontLeft.getPower() + "," + hw.backLeft.getPower() + "," +
                            hw.frontLeft.getCurrentPosition() + "," + hw.backLeft.getCurrentPosition() + "," +
                            (System.currentTimeMillis() - initTime));
                    Path.sleep_millisec_opmode(50, opmode);
                }
            }
        };
        loop.start();

        if (gamepad1.left_bumper) {     //SAMPLE IMPLEMENTATION
            StringBuilder sb = new StringBuilder();
            for (String row : kVData) {
                sb.append(row);
                sb.append("\n");
            }

            //DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/kV_regression_data_" + System.currentTimeMillis() + ".csv", sb.toString());
            //break;
        }
    }

    /*
    // maybe move these status teleop to main thread
                        // telemetry.addData("Current Drive Mode", "TOBY MODE");
                        if (!(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)) {

                            double speed;

                            if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                                speed = 0;
                            } else if (gamepad1.left_stick_y == 0) {
                                speed = Math.abs(gamepad1.left_stick_x);
                            } else if (gamepad1.left_stick_x == 0) {
                                speed = Math.abs(gamepad1.left_stick_y);
                            } else {
                                speed = Math.min(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y), 1);
                            }

                            double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                            drivetrain.MoveAngle(speed, angle, 0);
                        }

                        if (gamepad1.right_stick_x != 0) {
                            drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
                            if (gamepad1.right_stick_x > 0)
                                drivetrain.rotate(turnSpeed * gamepad1.right_stick_x, true);
                            else if (gamepad1.right_stick_x < 0)
                                drivetrain.rotate(turnSpeed * Math.abs(gamepad1.right_stick_x), false);
                            drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
                        }

                        if (gamepad1.left_trigger >= 0.5) {
                            drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
                            drivetrain.setSpeedMultiplier(normalSpeed - 0.3);
                        } else {
                            drivetrain.setSpeedMultiplier(TeleopConstants.drivePowerNormal);
                        }

                        if (gamepad1.right_trigger >= 0.75) {
                            drivetrain.setSpeedMultiplier(turboSpeed);
                        } else {
                            drivetrain.setSpeedMultiplier(normalSpeed + 0.2);
                        }

                        if (gamepad1.dpad_up) {
                            drivetrain.setPowerAll(slowSpeed - 0.2);
                        } else if (gamepad1.dpad_down) {
                            drivetrain.setPowerAll(-(slowSpeed - 0.2));
                        } else if (gamepad1.dpad_right) {
                            drivetrain.strafe(slowSpeed, true);
                        } else if (gamepad1.dpad_left) {
                            drivetrain.strafe(slowSpeed, false);
                        }

                        if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 &&
                                gamepad1.right_stick_y == 0 && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right
                                && !gamepad1.dpad_down) {
                            drivetrain.stop();
                        }
     */
}
