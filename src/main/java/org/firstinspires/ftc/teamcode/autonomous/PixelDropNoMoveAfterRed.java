package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Pixel Drop No Move After Red Side", group="Iterative OpMode")
//@Disabled
public class PixelDropNoMoveAfterRed extends BaseAuto {
    @Override
    public void init() {
        super.init();
        teamScoringElementFinder = new TSEVisionProcessorTesting(telemetry, true);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), teamScoringElementFinder, aprilTagProcessor);
        onRedTeam = true;
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
