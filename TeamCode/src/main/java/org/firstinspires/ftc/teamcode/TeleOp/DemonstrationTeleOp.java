package org.firstinspires.ftc.teamcode.TeleOp;

import android.webkit.WebHistoryItem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.All.Lift;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.GamepadButtons;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.OnOffButton;

import java.util.ArrayList;

@TeleOp(name = "DemonstrationTeleOp", group = "TeleOp")
public class DemonstrationTeleOp extends LinearOpMode {
    private Lift lift;
    private HardwareMap hwMap;
    private ArrayList<OnOffButton> buttonLogic = new ArrayList<>();
    private FourWheelMecanumDrivetrain drivetrain;
    private final double normalSpeed = TeleopConstants.drivePowerNormal;

    private boolean blockerRight = false;
    private boolean blockerLeft = false;
    private boolean initBlocker = false;
    private boolean run = true;

    private boolean intake = false;
    private boolean outake = false;
    private boolean blocker = false;

    private int autoClawStage = 0;
    private boolean isRed = true;

    private boolean manualOverride = false;
    private boolean capstoneBlock = false;

    @Override
    public void runOpMode(){
        hwMap = new HardwareMap(hardwareMap);

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

        while(!isStarted()){
            if(gamepad1.a)
                isRed = true;
            else if(gamepad1.b)
                isRed = false;

            telemetry.addData("Instructions", "Press A to select RED SIDE. Press B to select BLUE SIDE.");
            telemetry.addData("Currently Selected AutoClaw Side", isRed ? "RED" : "BLUE");
            telemetry.update();
        }

        waitForStart();

        liftLoop();
        toggleLoop();
        autoClawThread();
        initIntakeClaw();
        armDelayHandler();

        buttonLogic.add(new OnOffButton(gamepad1, gamepad1, GamepadButtons.A, GamepadButtons.B, //Intake-A & B
                new DcMotor[]{hwMap.leftIntake, hwMap.rightIntake},
                new double[][]{{-TeleopConstants.intakePower, 0}, {TeleopConstants.intakePower, 0}},
                new double[]{TeleopConstants.intakePower, -TeleopConstants.intakePower}));
        buttonLogic.add(new OnOffButton(gamepad1, GamepadButtons.Y, new Servo[]{hwMap.foundationLock, hwMap.transferLock},   //Foundation Lock-Y
                new double[][]{{TeleopConstants.foundationLockLock, TeleopConstants.foundationLockUnlock},
                        {TeleopConstants.transferLockPosUp, TeleopConstants.transferLockPosOut}}));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.LEFT_TRIGGER, new Servo[]{hwMap.transferHorn},
                new double[][]{{TeleopConstants.transferHornPosPush, TeleopConstants.transferHornPosReady}}));

        while (opModeIsActive()){
            if(gamepad1.right_trigger >= 0.5 && !blockerRight){
                blockerRight = true;
                if(autoClawStage + 1 <= 4)
                    autoClawStage += 1;
                else
                    autoClawStage = 0;
                run = true;
            } else if(gamepad1.right_trigger < 0.5 && blockerRight)
                blockerRight = false;

            if(gamepad1.dpad_up && !capstoneBlock){
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Capstone);
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2CapstoneOld);

                try{
                    Thread.sleep(300);
                } catch (Exception e){}

                hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosOpen);
            } else if(!gamepad1.dpad_up && capstoneBlock)
                capstoneBlock = false;

            /*if(gamepad1.left_trigger >= 0.5 && !blockerLeft){
                blockerLeft = true;
                autoClawStage = 0;
                run = true;
            } else if(gamepad1.right_trigger < 0.5 && blockerLeft)
                blockerLeft = false;*/

        }
    }

    private void liftLoop() {
        Thread drive = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad1.right_stick_y != 0) {
                        lift.moveLift(gamepad1.right_stick_y);
                    }  else {
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

    private void autoClawThread(){
        Thread t = new Thread(){
            public void run() {
                while (opModeIsActive()) {
                    switch (autoClawStage) {
                        case 0:
                            if(run) {
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
                            if(run) {
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
                            if(run) {
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
                            if(run) {
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
                            if(run) {
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

    private void initIntakeClaw(){
        Thread t2 = new Thread(){
            public void run(){
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
            }
        };
        Thread t = new Thread(){
            public void run(){
                while (opModeIsActive()){
                    if(gamepad1.x && !initBlocker){
                        t2.start();
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

                        buttonLogic.get(2).manualActivate(true, false);

                        try {
                            Thread.sleep(700);
                        } catch (Exception e) {
                        }

                        buttonLogic.get(2).manualActivate(true, false);

                        try {
                            Thread.sleep(700);
                        } catch (Exception e) {
                        }

                        buttonLogic.get(1).manualActivate(true, false);

                        try {
                            Thread.sleep(100);
                        } catch (Exception e) {
                        }

                        manualOverride = true;

                        try {
                            Thread.sleep(100);
                        } catch (Exception e) {
                        }
                    } else if(!gamepad1.x && initBlocker)
                        initBlocker = false;
                }
            }
        };
        t.start();
    }

    private void armDelayHandler() {
        Thread armDelay = new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad1.left_bumper && !blocker) {
                        if (!intake) {
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                            intake = true;
                            outake = false;
                        } else {
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Prep);
                            try {
                                Thread.sleep(250);
                            } catch (Exception e) {
                            }
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                            try {
                                Thread.sleep(250);
                            } catch (Exception e) {
                            }
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                            intake = false;
                            outake = false;
                        }
                        blocker = true;
                    }

                    if ((gamepad1.right_bumper || manualOverride) && !blocker) {
                        manualOverride = false;
                        if (!outake) {
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosReceive);
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2Block);
                            intake = false;
                            outake = true;
                        } else {
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1Prep);
                            try {
                                Thread.sleep(250);
                            } catch (Exception e) {
                            }
                            hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);
                            try {
                                Thread.sleep(250);
                            } catch (Exception e) {
                            }
                            hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
                            intake = false;
                            outake = false;
                        }
                        blocker = true;
                    }

                    if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                        blocker = false;
                    }
                }
            }
        };
        armDelay.start();
    }
}
