package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpCenterStage",group="Iterative OpMode")
public class TeleOpCenterStage extends OpMode {
    private final double StickDeadZone = 0.05;
    protected final ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected Servo clawDroneSideServo;
    protected Servo clawControlHubSideServo;
    protected DcMotor armMotor;
    protected DcMotor droneMotor;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;

    public static final double MIN_VALUE_FOR_WRIST_SERVO = 0.16;
    public static final double MAX_VALUE_FOR_WRIST_SERVO = 0.82;

    public static final double OPEN_VALUE_GRABBER_SERVO_DRONE_SIDE = 0.55;
    public static final double CLOSED_VALUE_GRABBER_SERVO_DRONE_SIDE = 0.66;

    public static final double OPEN_VALUE_GRABBER_SERVO_CONTROL_HUB_SIDE = 0.5;
    public static final double CLOSED_VALUE_GRABBER_SERVO_CONTROL_HUB_SIDE = 0.63;

    protected static final double CLOSED_VALUE_FOR_PIXEL_BACKDROPPER = 0.48;
    protected static final double OPEN_VALUE_FOR_PIXEL_BACKDROPPER = 0.33;

    double position;

//    final double ARM_SPEED_MULTIPLIER = 0.5;
    final double WRIST_SPEED = 0.005;
    //Once, when INIT is pressed

    private double forwards;
    private double sideways;
    private double turn;

    protected Servo autoBackdrop = null;
    @Override
    public void init() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        clawDroneSideServo = hardwareMap.get(Servo.class, "grabberServoDroneSide");
        clawControlHubSideServo = hardwareMap.get(Servo.class, "grabberServoControlHubSide");

        clawControlHubSideServo.setDirection(Servo.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");

        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        position = MIN_VALUE_FOR_WRIST_SERVO;

        autoBackdrop = hardwareMap.get(Servo.class, "autoBackdrop");

        telemetry.addData("Status","Initialized");
    }

    @Override
    public void start() {
        autoBackdrop.setPosition(CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);
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
        forwards = -gamepad1.left_stick_y;
        sideways = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

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

//      Shaping inputs
        shapeInput();

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

    private void shapeInput() {
        int fneg = 1;
        int sneg = 1;
        int tneg = 1;
        if (forwards<0) {
            fneg*=-1;
        }
        if (sideways<0) {
            sneg*=-1;
        }
        if (turn<0) {
            tneg*=-1;
        }

        forwards = Math.pow(forwards, 2);
        sideways = Math.pow(sideways, 2);
        turn = Math.pow(turn, 2);

        forwards *= fneg;
        sideways *= sneg;
        turn *= tneg;
    }
    private void hand() {
        if (gamepad2.left_bumper) {
            clawDroneSideServo.setPosition(OPEN_VALUE_GRABBER_SERVO_DRONE_SIDE);
        } else if (gamepad2.left_trigger >= 0.25) {
            clawDroneSideServo.setPosition(CLOSED_VALUE_GRABBER_SERVO_DRONE_SIDE);
        }

//        telemetry.addData("pos",
//                clawDroneSideServo.getPosition() + " " + clawControlHubSideServo.getPower());

        telemetry.addData("controls",
                "lb: " + gamepad2.left_bumper + " lt: " + (gamepad2.left_trigger>=.25) + " rb: " + gamepad2.right_bumper + " rt: " + (gamepad2.right_trigger>=.25));

        if (gamepad2.right_bumper) {
            clawControlHubSideServo.setPosition(OPEN_VALUE_GRABBER_SERVO_CONTROL_HUB_SIDE);
        } else if (gamepad2.right_trigger >= 0.25) {
            clawControlHubSideServo.setPosition(CLOSED_VALUE_GRABBER_SERVO_CONTROL_HUB_SIDE);
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
        if (gamepad2.a) {
            wristServoDroneSide.setPosition(0.46);
            wristServoControlHubSide.setPosition(0.46);
        }
        if (gamepad1.y) {
            autoBackdrop.setPosition(OPEN_VALUE_FOR_PIXEL_BACKDROPPER);
        }else {
            autoBackdrop.setPosition(CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);
        }

        telemetry.addData("survo pos",
                wristServoDroneSide.getPosition());
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }
}
