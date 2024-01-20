package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester")
@Disabled
public class ServoTester extends OpMode {
    private Servo pixelDropper;
    private Servo clawDroneSideServo;
    private Servo clawControlHubSideServo;
    private Servo autoBackdropPixelPlacer;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;

    private DcMotor droneMotor = null;
    private boolean dPadPressed = false;
    private boolean bumperPressed = false;
    private double openPos = 0.42;
    private double closedPos = 0.50;
    private double servoPos = closedPos;

    @Override
    public void init() {
        pixelDropper = hardwareMap.get(Servo.class, "pixelDropperServo");
        clawDroneSideServo = hardwareMap.get(Servo.class, "grabberServoDroneSide");
        clawControlHubSideServo = hardwareMap.get(Servo.class, "grabberServoControlHubSide");

        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");
        autoBackdropPixelPlacer = hardwareMap.get(Servo.class, "autoBackdrop");

        clawControlHubSideServo.setDirection(Servo.Direction.REVERSE);
//        pixelDropper.setPosition(servoPos);

//        autoBackdropPixelPlacer.setPosition(closedPos);

        wristServoControlHubSide.setPosition(0.5);
        wristServoDroneSide.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoPos = openPos;
        }else if (gamepad1.b) {
            servoPos = closedPos;
        }

        if (gamepad1.dpad_up && !dPadPressed) {
            if (openPos < 1.0) {
                openPos += 0.01;
            }
        } else if (gamepad1.dpad_down && !dPadPressed) {
            if (openPos > 0.0) {
                openPos -= 0.01;
            }
        }

        if (gamepad1.right_bumper && !bumperPressed) {
            if (closedPos < 1.0) {
                closedPos += 0.01;
            }
        } else if (gamepad1.left_bumper && !bumperPressed) {
            if (closedPos > 0.0) {
                closedPos -= 0.01;
            }
        }

        dPadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
        bumperPressed = gamepad1.right_bumper || gamepad1.left_bumper;

        autoBackdropPixelPlacer.setPosition(servoPos);

//        pixelDropper.setPosition(servoPos);
//        clawDroneSideServo.setPosition(servoPos);
//        rightArmServo.setPosition(servoPos);

//        wristServoControlHubSide.setPosition(servoPos);
//        wristServoDroneSide.setPosition(servoPos);

//        if (gamepad1.y) {
//            clawDroneSideServo.setPosition(servoPos);
//        }else if (gamepad1.x) {
//            clawControlHubSideServo.setPosition(servoPos);
//        }

        telemetry.addData("Servo position", servoPos);
        telemetry.addData("Open Pos", openPos);
        telemetry.addData("Closed Pos", closedPos);


    }
}
