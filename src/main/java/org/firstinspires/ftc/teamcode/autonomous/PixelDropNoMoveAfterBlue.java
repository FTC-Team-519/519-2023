package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Pixel Drop No Move After Blue Side", group="Iterative OpMode")
//@Disabled
public class PixelDropNoMoveAfterBlue extends BaseAuto {
    @Override
    public void init() {
        super.init();
        teamScoringElementFinder = new TSEVisionProcessorTesting(telemetry, false);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), teamScoringElementFinder, aprilTagProcessor);
        onRedTeam = false;
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {}
}
