package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseOpmode extends OpMode {
    protected final ElapsedTime runtime = new ElapsedTime();

    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;

    //Once, when INIT is pressed
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
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
        telemetry.addData("Status", "Run Time: %s", runtime.toString());
    }

    //Once when STOP is pressed
    @Override
    public void stop() {}
}
