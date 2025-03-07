package org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class OnOffButton {

    private enum Situation {
        ONE_BUTTON_MOTOR, ONE_BUTTON_SERVO, TWO_BUTTON_MOTOR, TWO_BUTTON_SERVO
    }

    private DcMotor[] motors;
    private Servo[] servos;
    private double[][] powOrPos1;
    private double[] powOrPos2;
    private boolean blocker;
    private GamepadButtons button1;
    private GamepadButtons button2;
    private Gamepad gamepadSituation1;
    private Gamepad gamepadSituation2;
    private Situation situation;

    private boolean mvmt1 = false;
    private boolean mvmt2 = false;

    private boolean manualActivate1 = false;
    private boolean manualActivate2 = false;

    public OnOffButton(Gamepad gamepad, GamepadButtons toggleButton, DcMotor[] motors,
                       double[][] power) {
        this.motors = motors;
        this.gamepadSituation1 = gamepad;
        powOrPos1 = power;
        button1 = toggleButton;
        blocker = false;
        situation = Situation.ONE_BUTTON_MOTOR;
    }

    public OnOffButton(Gamepad gamepad, GamepadButtons toggleButton, Servo[] servos,
                       double[][] positions) {
        this.servos = servos;
        this.gamepadSituation1 = gamepad;
        powOrPos1 = positions;
        button1 = toggleButton;
        blocker = false;
        situation = Situation.ONE_BUTTON_SERVO;
    }

    public OnOffButton(Gamepad gamepadNo1, Gamepad gamepadNo2, GamepadButtons toggleButton1, GamepadButtons toggleButton2,
                       DcMotor[] motors, double[][] power1, double[] power2) {
        this.motors = motors;
        gamepadSituation1 = gamepadNo1;
        gamepadSituation2 = gamepadNo2;
        powOrPos1 = power1;
        powOrPos2 = power2;
        button1 = toggleButton1;
        button2 = toggleButton2;
        blocker = false;
        situation = Situation.TWO_BUTTON_MOTOR;
    }

    public OnOffButton(Gamepad gamepadNo1, Gamepad gamepadNo2, GamepadButtons toggleButton1, GamepadButtons toggleButton2,
                       Servo[] servos, double[][] position1, double[] position2) {
        this.servos = servos;
        gamepadSituation1 = gamepadNo1;
        gamepadSituation2 = gamepadNo2;
        powOrPos1 = position1;
        powOrPos2 = position2;
        button1 = toggleButton1;
        button2 = toggleButton2;
        blocker = false;
        situation = Situation.TWO_BUTTON_SERVO;
    }

    public void getGamepadStateAndRun(){
        if(situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            oneButtonHandler();
        else if(situation == Situation.TWO_BUTTON_MOTOR || situation == Situation.TWO_BUTTON_SERVO)
            twoButtonHandler();
    }

    public boolean[] getState(){
        if(situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            return new boolean[] { mvmt1 };
        else
            return new boolean[] { mvmt1, mvmt2 };
    }

    public GamepadButtons[] getButtons(){
        if(situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.ONE_BUTTON_SERVO)
            return new GamepadButtons[] { button1 };
        else
            return new GamepadButtons[] { button1, button2 };
    }

    public boolean isControllingServos(){
        return situation == Situation.ONE_BUTTON_SERVO || situation == Situation.TWO_BUTTON_SERVO;
    }

    public boolean isControllingDcMotors(){
        return situation == Situation.ONE_BUTTON_MOTOR || situation == Situation.TWO_BUTTON_MOTOR;
    }

    public Servo[] getServos(){
        return servos;
    }

    public DcMotor[] getMotors(){
        return motors;
    }
	
	public void manualActivate(boolean manualActivate1, boolean manualActivate2){
		this.manualActivate1 = manualActivate1;
		this.manualActivate2 = manualActivate2;
	}

    private void oneButtonLogic(){
        if(!blocker){
            blocker = true;
            manualActivate1 = false;
            manualActivate2 = false;
            mvmt1 = !mvmt1;
        }
    }

    private void twoButtonLogic1() {
        if (!blocker) {
            if (!mvmt1) {
                mvmt1 = true;
                mvmt2 = false;
                manualActivate1 = false;
                manualActivate2 = false;
            } else {
                mvmt1 = false;
                mvmt2 = false;
            }
            blocker = true;
        }
    }

    private void twoButtonLogic2() {
        if (!blocker) {
            if (!mvmt2) {
                mvmt1 = false;
                mvmt2 = true;
                manualActivate1 = false;
                manualActivate2 = false;
            } else {
                mvmt1 = false;
                mvmt2 = false;
            }
            blocker = true;
        }
    }

    private void oneButtonServoLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][0]);
            }
        }

        if (!mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][1]);
            }
        }
    }

    private void oneButtonMotorLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][0]);
            }
        }

        if (!mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][1]);
            }
        }
    }

    private void twoButtonServoLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][0]);
            }
        }

        if (mvmt2) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos2[i]);
            }
        }

        if (!mvmt1 && !mvmt2) {
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(powOrPos1[i][1]);
            }
        }
    }

    private void twoButtonMotorLogicAdditional() {
        if (mvmt1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][0]);
            }
        }

        if (mvmt2) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos2[i]);
            }
        }

        if (!mvmt1 && !mvmt2) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(powOrPos1[i][1]);
            }
        }
    }

    //------------------==Button Handlers (Don't look, it'll hurt your brain at how inefficient this is)==------------------

    /* Are you ready of > 1000 lines of copy pasta code (To handle each case of the enum)? */

    private void oneButtonHandler() {
        switch (button1) {
            case A:
                if(gamepadSituation1.a || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.a && !manualActivate1)
                    blocker = false;
                break;
            case B:
                if(gamepadSituation1.b || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.b && !manualActivate1)
                    blocker = false;
                break;
            case X:
                if(gamepadSituation1.x || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.x && !manualActivate1)
                    blocker = false;
                break;
            case Y:
                if(gamepadSituation1.y || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.y && !manualActivate1)
                    blocker = false;
                break;
            case DPAD_DOWN:
                if(gamepadSituation1.dpad_down || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.dpad_down && !manualActivate1)
                    blocker = false;
                break;
            case DPAD_LEFT:
                if(gamepadSituation1.dpad_left || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.dpad_left && !manualActivate1)
                    blocker = false;
                break;
            case DPAD_RIGHT:
                if(gamepadSituation1.dpad_right || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.dpad_right && !manualActivate1)
                    blocker = false;
                break;
            case DPAD_UP:
                if(gamepadSituation1.dpad_up || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.dpad_up && !manualActivate1)
                    blocker = false;
                break;
            case LEFT_BUMPER:
                if(gamepadSituation1.left_bumper || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.left_bumper && !manualActivate1)
                    blocker = false;
                break;
            case RIGHT_BUMPER:
                if(gamepadSituation1.right_bumper || manualActivate1)
                    oneButtonLogic();
                else if(!gamepadSituation1.right_bumper && !manualActivate1)
                    blocker = false;
                break;
            case LEFT_TRIGGER:
                if(gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                    oneButtonLogic();
                else if(gamepadSituation1.left_trigger < 0.5 && !manualActivate1)
                    blocker = false;
                break;
            case RIGHT_TRIGGER:
                if(gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                    oneButtonLogic();
                else if(gamepadSituation1.right_trigger < 0.5 && !manualActivate1)
                    blocker = false;
                break;
        }
        if(situation == Situation.ONE_BUTTON_MOTOR)
            oneButtonMotorLogicAdditional();
        else if(situation == Situation.ONE_BUTTON_SERVO)
            oneButtonServoLogicAdditional();
    }

    private void twoButtonHandler() {
        switch (button1) {
            case A:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.a || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.a && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case B:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.b || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.b && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case X:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.x || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.x && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case Y:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.y || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.y && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case DPAD_DOWN:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.dpad_down || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_down && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case DPAD_LEFT:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.dpad_left || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_left && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case DPAD_RIGHT:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.dpad_right || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_right && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case DPAD_UP:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.dpad_up)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.dpad_up || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.dpad_up && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case LEFT_BUMPER:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.left_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.left_bumper && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case RIGHT_BUMPER:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.right_bumper || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.right_bumper)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (!gamepadSituation1.right_bumper && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case LEFT_TRIGGER:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.left_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.left_trigger < 0.5 && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
            case RIGHT_TRIGGER:
                switch (button2) {
                    case A:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.a || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.a && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case B:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.b || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.b && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case X:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.x || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.x && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case Y:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.y || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.y && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_DOWN:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_down || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.dpad_down && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_LEFT:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_left || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.dpad_left && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_RIGHT:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_right || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.dpad_right && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case DPAD_UP:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.dpad_up || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.dpad_up && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_BUMPER:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.left_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_BUMPER:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_bumper || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && !gamepadSituation2.right_bumper && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case LEFT_TRIGGER:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.left_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && gamepadSituation2.left_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                    case RIGHT_TRIGGER:
                        if (gamepadSituation1.right_trigger >= 0.5 || manualActivate1)
                            twoButtonLogic1();

                        if (gamepadSituation2.right_trigger >= 0.5 || manualActivate2)
                            twoButtonLogic2();

                        if (gamepadSituation1.right_trigger < 0.5 && gamepadSituation2.right_trigger < 0.5 && !manualActivate1 && !manualActivate2)
                            blocker = false;
                        break;
                }
                break;
        }
        if (situation == Situation.TWO_BUTTON_MOTOR)
            twoButtonMotorLogicAdditional();
        else if (situation == Situation.TWO_BUTTON_SERVO)
            twoButtonServoLogicAdditional();
    }
}
