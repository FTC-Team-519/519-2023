package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester")
//@Disabled
public class ServoTester extends OpMode {
    private Servo pixelDropper;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private Servo autoBackdropPixelPlacer;
    private DcMotor droneMotor = null;
    private boolean dPadPressed = false;
    private boolean bumperPressed = false;
    private double openPos = 0.42;
    private double closedPos = 0.57;
    private double servoPos = closedPos;

    @Override
    public void init() {
//        pixelDropper = hardwareMap.get(Servo.class, "pixelDropperServo");
//        leftArmServo = hardwareMap.get(Servo.class, "armServoDroneSide");
//        rightArmServo = hardwareMap.get(Servo.class, "armServoControlHubSide");
//        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");
        autoBackdropPixelPlacer = hardwareMap.get(Servo.class, "autoBackdrop");
//        rightArmServo.setDirection(Servo.Direction.REVERSE);
//        pixelDropper.setPosition(servoPos);\
        autoBackdropPixelPlacer.setPosition(closedPos);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoPos = openPos;
        }else if (gamepad1.b) {
            servoPos = closedPos;
        }

        if (gamepad1.dpad_up && !dPadPressed) {
            openPos += 0.01;
        } else if (gamepad1.dpad_down && !dPadPressed) {
            openPos -= 0.01;
        }

        if (gamepad1.right_bumper && !bumperPressed) {
            closedPos += 0.01;
        } else if (gamepad1.left_bumper && !bumperPressed) {
            closedPos -= 0.01;
        }

        dPadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
        bumperPressed = gamepad1.right_bumper || gamepad1.left_bumper;

        autoBackdropPixelPlacer.setPosition(servoPos);

//        pixelDropper.setPosition(servoPos);
//        leftArmServo.setPosition(servoPos);
//        rightArmServo.setPosition(servoPos);

        telemetry.addData("Servo position", servoPos);
        telemetry.addData("Open Pos", openPos);
        telemetry.addData("Closed Pos", closedPos);


    }
}
