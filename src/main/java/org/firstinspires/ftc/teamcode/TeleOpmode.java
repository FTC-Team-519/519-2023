package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanim Teleop",group="Iterative OpMode")
public class TeleOpmode extends BaseOpmode {

    @Override
    public void loop() {

        double max;
        double forwards = Math.pow(-gamepad1.left_stick_y, 3);
        double sideways = Math.pow(gamepad1.left_stick_x, 3);
        double turn = Math.pow(gamepad1.right_stick_x, 3);
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

        super.loop();

        telemetry.addData("Motors", "%.4f %.4f %.4f %.4f",
                frontRightPower, backRightPower, frontLeftPower, frontRightPower);
    }
}
