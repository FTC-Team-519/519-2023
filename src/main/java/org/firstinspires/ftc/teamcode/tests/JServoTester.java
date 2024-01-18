package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Iterator;

@TeleOp(name = "James's Servo Tester")
//@Disabled
public class JServoTester extends OpMode {

    private Servo[] servos;
    private int servoNumber = 0;
    private double pos;
    private double altPos;
    private double diff;
    @Override
    public void init() {
        ArrayList<Servo> servoList = new ArrayList<Servo>();
        for (Servo servo : hardwareMap.servo) {
            if (servo != null) servoList.add(servo);
        }
        if (servoList.isEmpty()) {
            telemetry.addData("Status", "No servos detected.");
            updateTelemetry(telemetry);
            requestOpModeStop();
            return;
        }
        servos = new Servo[servoList.size()];
        for (int i = 0; i < servos.length; i++) {
            servos[i] = servoList.get(i);
        }

        pos = 0.5;
        altPos = 0.5;
        diff = 0.1;
    }

    private boolean left = false;
    private boolean up = false;
    private boolean right = false;
    private boolean down = false;
    private boolean shifting = false;
    private boolean poschange = false;
    @Override
    public void loop() {
        if (poschange) {
            poschange = gamepad1.x;
        } else if (gamepad1.x) {
            double tmp = pos;
            pos = altPos;
            altPos = tmp;
            poschange = true;
        } else poschange = false;
        if (gamepad1.left_bumper) {
            if (!shifting) {
                if (gamepad1.right_bumper) {
                    shifting = true;
                }
                shifting = true;
                servoNumber = (servoNumber + (servos.length - 1)) % servos.length;
            }
        } else if (gamepad1.right_bumper) {
            if (!shifting) {
                shifting = true;
                servoNumber = (servoNumber + 1) % servos.length;
            }
        } else shifting = false;

        Servo servo = servos[servoNumber];

        if (gamepad1.a) servo.setPosition(pos);
        else if (gamepad1.b) servo.setPosition(altPos);
        if (gamepad1.dpad_left) {
            if (!left) pos -= diff;
            left = true;
        } else left = false;
        if (gamepad1.dpad_down) {
            if (!down) diff /= 10;
            down = true;
        } else down = false;
        if (gamepad1.dpad_right) {
            if (!right) pos += diff;
            right = true;
        } else right = false;
        if (gamepad1.dpad_up) {
            if (!up) diff *= 10;
            up = true;
        } else up = false;
        telemetry.addData("Servo #", "%d", servoNumber);
        telemetry.addData("Servo port #", "%d", servo.getPortNumber());
        telemetry.addData("Servo Name", "%s", hardwareMap.getNamesOf(servo)
                .stream().findFirst().orElse("Name Missing"));
        telemetry.addData("Servo Position", "%.4f", servo.getPosition());
        telemetry.addData("Target Position", "%.4f", pos);
        telemetry.addData("Alt Position", "%.4f", altPos);
        telemetry.addData("Change Amount", "%.4f", diff);
    }
}
