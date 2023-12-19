package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpCenterStage",group="Iterative OpMode")
public class autoBackDropTest extends OpMode {
    private final double StickDeadZone = 0.05;
    protected final ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected CRServo clawDroneSideServo;

    protected CRServo clawControlHubSideServo;
    protected DcMotor armMotor;
    protected DcMotor droneMotor;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;
    protected Servo autoBackdrop = null;
    protected static double CLOSED_VALUE_FOR_PIXEL_BACKDROPPER = 0.53;
    protected static double OPEN_VALUE_FOR_PIXEL_BACKDROPPER = 0.36;

    public static final double MIN_VALUE_FOR_WRIST_SERVO = 0.27;
    public static final double MAX_VALUE_FOR_WRIST_SERVO = 0.85;

    double position;

    //    final double ARM_SPEED_MULTIPLIER = 0.5;
    final double WRIST_SPEED = 0.005;
    //Once, when INIT is pressed

    boolean rbJustPressed = false;
    boolean lbJustPressed = false;
    boolean rtJustPressed = false;
    boolean ltJustPressed = false;

    @Override
    public void init() {
        autoBackdrop = hardwareMap.get(Servo.class, "autoBackdrop");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        clawDroneSideServo = hardwareMap.get(CRServo.class, "grabberServo1");
        clawControlHubSideServo = hardwareMap.get(CRServo.class, "grabberServo2");

        clawControlHubSideServo.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");

        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        position = MIN_VALUE_FOR_WRIST_SERVO;

        telemetry.addData("Status","Initialized");
    }
    @Override
    public void loop() {
        drive();
        hand();
        arm();

        launch();
        preset();
    }

    private void drive() {
        double max;
        double forwards = -gamepad1.left_stick_y;
        double sideways = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.dpad_up) {
            forwards = 1;
            sideways = 0;
            turn = 0;
        } else if (gamepad1.dpad_down) {
            forwards = -1;
            sideways = 0;
            turn = 0;
        } else if (gamepad1.dpad_left) {
            forwards = 0;
            sideways = -1;
            turn = 0;
        } else if (gamepad1.dpad_right) {
            forwards = 0;
            sideways = 1;
            turn = 0;
        }

//        Stick drift
//        if (forwards+StickDeadZone>=0 || forwards-StickDeadZone==0) {forwards = 0;}
//        if (sideways+StickDeadZone==0 || sideways-StickDeadZone==0) {sideways = 0;}
//        if (turn+StickDeadZone==0 || turn-StickDeadZone==0) {turn = 0;}

//      Shaping inputs
        forwards = Math.pow(forwards, 3);
        sideways = Math.pow(sideways, 3);
        turn = Math.pow(turn, 3);

        double frontLeftPower = forwards + sideways + turn;
        double frontRightPower = forwards - sideways - turn;
        double backLeftPower = forwards - sideways + turn;
        double backRightPower = forwards + sideways - turn;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }

    private void hand() {
        if (gamepad2.left_bumper) {
            clawDroneSideServo.setPower(1.0);
        } else if (gamepad2.left_trigger >= 0.25) {
            clawDroneSideServo.setPower(-1.0);
        } else {
            clawDroneSideServo.setPower(0.0);
        }

        telemetry.addData("pow",
                clawDroneSideServo.getPower() + " " + clawControlHubSideServo.getPower());

        telemetry.addData("controls",
                "lb: " + gamepad2.left_bumper + " lt: " + (gamepad2.left_trigger>=.25) + " rb: " + gamepad2.right_bumper + " rt: " + (gamepad2.right_trigger>=.25));

        if (gamepad2.right_bumper) {
            clawControlHubSideServo.setPower(1.0);
        } else if (gamepad2.right_trigger >= 0.25) {
            clawControlHubSideServo.setPower(-1.0);
        } else {
            clawControlHubSideServo.setPower(0.0);
        }

        if (gamepad2.x) {
            position = wristServoDroneSide.getPosition() + WRIST_SPEED;

            if (position > MAX_VALUE_FOR_WRIST_SERVO) {
                position = MAX_VALUE_FOR_WRIST_SERVO;
            }
            moveWrist(position);
        }

        if (gamepad2.b) {
            position = wristServoDroneSide.getPosition() - WRIST_SPEED;

            if (position < MIN_VALUE_FOR_WRIST_SERVO) {
                position = MIN_VALUE_FOR_WRIST_SERVO;
            }
            moveWrist(position);
        }

        telemetry.addData("position",
                position);

    }

    private void launch() {
        if (gamepad1.a && gamepad1.b) {
            droneMotor.setPower(-1);
        } else {
            droneMotor.setPower(0.0);
        }
    }

    private void arm() {
        double power = -gamepad2.left_stick_y;

//        if (gamepad1.x) {
//            armMotor.setPower(power);
//        } else {
//            armMotor.setPower(power * ARM_SPEED_MULTIPLIER);
//        }

        armMotor.setPower(power);

        telemetry.addData("armpos",
                armMotor.getCurrentPosition() + " " + power);
    }

    private void moveWrist(double position) {
        wristServoDroneSide.setPosition(position);
        wristServoControlHubSide.setPosition(position);
    }

    private void preset() {
        if (gamepad1.a) {
//            wristServoDroneSide.setPosition(0.6);
//            wristServoControlHubSide.setPosition(0.6);
            autoBackdrop.setPosition(CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);
        } else if (gamepad1.b) {
//            wristServoDroneSide.setPosition(0.6);
//            wristServoControlHubSide.setPosition(0.6);
            autoBackdrop.setPosition(OPEN_VALUE_FOR_PIXEL_BACKDROPPER);
        }

        if (gamepad1.right_bumper && !rbJustPressed) {
            OPEN_VALUE_FOR_PIXEL_BACKDROPPER = autoBackdrop.getPosition() +.01;
        } else if (gamepad1.left_bumper && !lbJustPressed) {
            OPEN_VALUE_FOR_PIXEL_BACKDROPPER = autoBackdrop.getPosition() -.01;
        }

        if (gamepad1.right_trigger > 0.5 && !rtJustPressed) {
            CLOSED_VALUE_FOR_PIXEL_BACKDROPPER = autoBackdrop.getPosition() +.01;
        } else if (gamepad1.left_trigger > 0.5 && !ltJustPressed) {
            CLOSED_VALUE_FOR_PIXEL_BACKDROPPER = autoBackdrop.getPosition() -.01;
        }

        rbJustPressed = gamepad1.right_bumper;
        lbJustPressed = gamepad1.left_bumper;
        rtJustPressed = gamepad1.right_trigger > 0.5;
        ltJustPressed = gamepad1.left_trigger > 0.5;

        telemetry.addData("auto backdrop droper open",
                armMotor.getCurrentPosition() + " " + OPEN_VALUE_FOR_PIXEL_BACKDROPPER);
        telemetry.addData("auto backdrop droper close",
                armMotor.getCurrentPosition() + " " + CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);
    }


    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
}
