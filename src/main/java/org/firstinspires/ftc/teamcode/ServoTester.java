package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester")
@Disabled
public class ServoTester extends OpMode {
    private Servo pixelDropper;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor droneMotor = null;
    private double servoPos = 0.5;
    private boolean dPadPressed = false;
    private double power = 0;

    @Override
    public void init() {
//        pixelDropper = hardwareMap.get(Servo.class, "pixelDropper");
        leftArmServo = hardwareMap.get(Servo.class, "armServoDroneSide");
        rightArmServo = hardwareMap.get(Servo.class, "armServoControlHubSide");
        droneMotor = hardwareMap.get(DcMotor.class, "droneMotor");
        rightArmServo.setDirection(Servo.Direction.REVERSE);
//        pixelDropper.setPosition(servoPos);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            power = 1.0;
        }else {
            power = 0.0;
        }

        if (gamepad1.dpad_up && !dPadPressed) {
            servoPos += 0.01;
        }
        if (gamepad1.dpad_down && !dPadPressed) {
            servoPos -= 0.01;
        }
        if (servoPos > 1.0) {
            servoPos = 1.0;
        }
        if (servoPos < 0.0) {
            servoPos = 0.0;
        }
        dPadPressed = gamepad1.dpad_up || gamepad1.dpad_down;
//        pixelDropper.setPosition(servoPos);
        leftArmServo.setPosition(servoPos);
        rightArmServo.setPosition(servoPos);
        droneMotor.setPower(power);

        telemetry.addData("Left Arm Servo Position", leftArmServo.getPosition());
        telemetry.addData("Right Arm Servo Position", rightArmServo.getPosition());
    }
}
