package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Do Nothing", group="group")
public class DoNothing extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        telemetry.addData("Status","Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: %s", runtime.toString());
    }
}
