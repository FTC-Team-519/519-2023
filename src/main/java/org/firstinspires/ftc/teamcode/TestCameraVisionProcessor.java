package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Test Camera Vision Processor", group = "Telop")
public class TestCameraVisionProcessor extends OpMode {
    private TestVisionProcessor visionProcessor;
    private VisionPortal portal;
    @Override
    public void init() {
        visionProcessor = new TestVisionProcessor(telemetry);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        portal.close();
    }
}
