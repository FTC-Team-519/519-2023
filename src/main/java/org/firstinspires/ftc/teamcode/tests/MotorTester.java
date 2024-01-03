package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MotorTesterThing",group="Iterative OpMode")
public class MotorTester extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

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

        telemetry.addData("Status","Initialized");
    }

    //Repeatedly, once INIT is pressed
    @Override
    public void init_loop() {}

    //Once, when PLAY is pressed
    @Override
    public void start() {
        runtime.reset();
    }
    //Repeatedly, once PLAY is pressed
    @Override
    public void loop() {

        double max;
        double frontLeftPower = Range.clip(-gamepad1.left_stick_y, -1, 1);
        double backLeftPower = Range.clip(gamepad1.left_stick_x, -1, 1);
        double frontRightPower = Range.clip(-gamepad1.right_stick_y, -1, 1);
        double backRightPower = Range.clip(gamepad1.right_stick_x, -1, 1);

        /*max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }*/
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Status", "Run Time: %s", runtime.toString());
        telemetry.addData("Front Right Motor", frontRightPower);
        telemetry.addData("Back Right Motor", backRightPower);
        telemetry.addData("Front Left Motor", frontLeftPower);
        telemetry.addData("Back Left Motor", backLeftPower);
        telemetry.addData("Front Right Encoder", frontRightMotor.getCurrentPosition());
        telemetry.addData("Back Right Encoder", backRightMotor.getCurrentPosition());
        telemetry.addData("Front Left Encoder", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Back Left Encoder", backLeftMotor.getCurrentPosition());
//                telemetry.addData("Encoders", "(fr,br,fl,bl) (%.3f %.3f %.3f %.3f)",
//            frontRightMotor.getCurrentPosition(), backRightMotor.getCurrentPosition(),
//            frontLeftMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition());

    }

    //Once when STOP is pressed
    @Override
    public void stop() {}
}
