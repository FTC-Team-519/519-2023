package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpCenterStage",group="Iterative OpMode")
public class TeleOpCenterStage extends OpMode {
    private final double StickDeadZone = 0.05;
    protected final ElapsedTime runtime = new ElapsedTime();
    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;
    protected CRServo clawDroneSideServo;

    protected CRServo clawControlHubSideServo;
    protected DcMotor armMotor;
    protected DcMotor droneMotor;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;

    public static final double MIN_VALUE_FOR_WRIST_SERVO = 0.27;
    public static final double MAX_VALUE_FOR_WRIST_SERVO = 0.85;

    double position;
    //Once, when INIT is pressed
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

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
            sideways = 1;
            turn = 0;
        } else if (gamepad1.dpad_right) {
            forwards = 0;
            sideways = -1;
            turn = 0;
        }

//        Stick drift
//        if (forwards+StickDeadZone>=0 || forwards-StickDeadZone==0) {forwards = 0;}
//        if (sideways+StickDeadZone==0 || sideways-StickDeadZone==0) {sideways = 0;}
//        if (turn+StickDeadZone==0 || turn-StickDeadZone==0) {turn = 0;}

//      Shaping inputs
//        forwards = Math.pow(forwards, 3);
//        sideways = Math.pow(sideways, 3);
//        turn = Math.pow(turn, 3);

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
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
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
            position = wristServoDroneSide.getPosition() + 0.005;

            if (position > MAX_VALUE_FOR_WRIST_SERVO) {
                position = MAX_VALUE_FOR_WRIST_SERVO;
            }
            moveWrist(position);
        }

        if (gamepad2.b) {
            position = wristServoDroneSide.getPosition() - 0.005;

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
            droneMotor.setPower(1.0);
        } else {
            droneMotor.setPower(0.0);
        }
    }

    private void arm() {
        double power = -gamepad2.left_stick_y;
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
            wristServoDroneSide.setPosition(0.6);
            wristServoControlHubSide.setPosition(0.6);
        }
    }
}
