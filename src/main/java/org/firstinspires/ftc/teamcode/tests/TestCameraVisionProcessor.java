package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.TSEVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Test Camera Vision Processor", group = "Telop")
@Disabled
public class TestCameraVisionProcessor extends OpMode {
    private org.firstinspires.ftc.teamcode.autonomous.TSEVisionProcessor TSEVisionProcessor;
    private VisionPortal portal;
    private Gamepad driver;
    @Override
    public void init() {
        TSEVisionProcessor = new TSEVisionProcessor(telemetry, true);
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), TSEVisionProcessor);
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
