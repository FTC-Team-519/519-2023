package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester")
public class ServoTester extends OpMode {
    private Servo pixelDropper;
    private double servoPos = 0.5;
    private boolean dPadPressed = false;
    @Override
    public void init() {
        pixelDropper = hardwareMap.get(Servo.class, "pixelDropper");
        pixelDropper.setPosition(servoPos);
    }

    @Override
    public void loop() {
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
        pixelDropper.setPosition(servoPos);

        telemetry.addData("Servo Position", pixelDropper.getPosition());
    }
}
