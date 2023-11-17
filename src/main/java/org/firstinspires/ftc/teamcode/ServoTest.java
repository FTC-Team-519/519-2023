package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ServoTest", group="Tests")
public class ServoTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private Servo servo;

    //Once, when INIT is pressed
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");

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
        servo.setPosition(Range.clip((gamepad1.left_stick_x + 1) / 2, 0.0, 1.0));
        telemetry.addData("Status", "Run Time: %s", runtime.toString());
    }

    //Once when STOP is pressed
    @Override
    public void stop() {}
}
