package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class BaseAuto extends OpMode {
    protected boolean USE_WEBCAM = true;
    protected TestVisionProcessor visionProcessor;
    protected VisionPortal portal;
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected AprilTagProcessor aprilTagProcessor;


    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            portal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
        } else {
            portal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTagProcessor);
        }


    }
}
