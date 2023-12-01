package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class HopefullyDrive extends BaseOpmode {
	private IMU imu;

	private final TestVisionProcessor visionProcessor;

	protected HopefullyDrive(boolean isRed) {
		super();
		visionProcessor = new TestVisionProcessor(telemetry, isRed);
	}

	@Override
	public void init() {
		super.init();

		imu = hardwareMap.get(IMU.class, "imu");

		imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

		setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
		frontLeftMotor.setZeroPowerBehavior(behavior);
		frontRightMotor.setZeroPowerBehavior(behavior);
		backLeftMotor.setZeroPowerBehavior(behavior);
		backRightMotor.setZeroPowerBehavior(behavior);
	}

	protected void setMode(DcMotor.RunMode mode) {
		frontLeftMotor.setMode(mode);
		frontRightMotor.setMode(mode);
		backLeftMotor.setMode(mode);
		backRightMotor.setMode(mode);
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
	public void stop() {
		super.stop();
	}
}
