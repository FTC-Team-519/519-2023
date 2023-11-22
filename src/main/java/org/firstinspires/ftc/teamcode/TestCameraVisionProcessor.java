package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Test Camera Vision Processor", group = "Telop")
public class TestCameraVisionProcessor extends OpMode {
    private TestVisionProcessor visionProcessor;
    private VisionPortal portal;
    private Gamepad gamepad1;
    @Override
    public void init() {
        visionProcessor = new TestVisionProcessor(telemetry, true);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        portal.resumeStreaming();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        portal.close();
    }
}
