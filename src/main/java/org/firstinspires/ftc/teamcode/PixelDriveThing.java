/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Red Autonomous Pixel Drop", group="Autonomous")
public class PixelDriveThing extends LinearOpMode {

	public PixelDriveThing() {
		this(true);
	}

	public PixelDriveThing(boolean red) {

	}

	private TestVisionProcessor visionProcessor;
	private VisionPortal portal;

	/* Motors. */
	private DcMotor frontLeftMotor;
	private DcMotor backLeftMotor;
	private DcMotor frontRightMotor;
	private DcMotor backRightMotor;
	//IMU
	private IMU imu;
	private Servo servo;
	private ColorSensor colorSensor;

	private double headingError = 0;

	private double driveSpeed= 0;
	private double turnSpeed = 0;

	// Calculate the COUNTS_PER_INCH for your specific drive train.
	// Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
	// For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
	// For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
	// This is gearing DOWN for less speed and more torque.
	// For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
	// (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
	static final double COUNTS_PER_INCH = (1425.1 * 1.0) / (4.0 * 3.1415);

	// These constants define the desired driving/control characteristics
	// They can/should be tweaked to suit the specific robot drive train.
	static final double DRIVE_SPEED = 0.4; // Max driving speed for better distance accuracy.
	static final double TURN_SPEED = 0.2; // Max Turn speed to limit turn rate
	static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
	// Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
	// Define the Proportional control coefficient (or GAIN) for "heading control".
	// We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
	// Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
	// Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni-wheels)
	static final double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable
	static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable


	@Override
	public void runOpMode() {
		try {
			frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
			frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
			backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
			backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
			servo = hardwareMap.get(Servo.class, "pixelDropperServo");
			colorSensor = hardwareMap.get(ColorSensor.class, "pixelColorSensor");

			servo.setPosition(0.35);

			// Now initialize the IMU with this mounting orientation
			// This sample expects the IMU to be in a REV Hub and named "imu".
			imu = hardwareMap.get(IMU.class, "imu");
			imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
			));

			// Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
			frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
			backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
			frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
			backRightMotor.setDirection(DcMotor.Direction.FORWARD);
			setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

			visionProcessor = new TestVisionProcessor(telemetry, true);
			portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
			portal.resumeStreaming();

			// Wait for the game to start (Display Gyro value while waiting)
			while (opModeInInit()) {
				sleep(100);
			}
			if (!opModeIsActive()) return;
			sleep(5000); //Let the tse detection do its thing
			if (!opModeIsActive()) return;
			// Set the encoders for closed loop speed control, and reset the heading.
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			imu.resetYaw();


			boolean spikeLeft = visionProcessor.getLeft();
			boolean spikeCenter = visionProcessor.getCenter();
			boolean spikeRight = visionProcessor.getRight();

			if (!opModeIsActive()) return;
			driveStraight(DRIVE_SPEED, 15, 0);
			if (spikeLeft) {
				telemetry.addData("TSE", "left");
				telemetry.update();
				turnToHeading(TURN_SPEED, 90);
				//driveStraight(DRIVE_SPEED, 0, 45);
			} else if (spikeRight) {
				telemetry.addData("TSE", "right");
				telemetry.update();
				turnToHeading(TURN_SPEED, -90);
				//driveStraight(DRIVE_SPEED, 0, -45);
			} else if (spikeCenter) { //If the TSE isn't detected, pretend it's in the center
				telemetry.addData("TSE", "center");
				telemetry.update();
				//driveStraight(DRIVE_SPEED, 0, 0);
			} else {
				telemetry.addData("TSE", "not found");
				telemetry.update();
			}
			if (!opModeIsActive()) return;
			servo.setPosition(0.38);
			if (opModeIsActive()) sleep(100);  // Pause to display last telemetry message.
		} finally { //Stuff to run on exit. This way, return can be used in the body.
			portal.close();
		}
	}

	/**
	 * Convenience method for setting motor modes
	 */
	public void setMode(DcMotor.RunMode mode) {
		frontLeftMotor.setMode(mode);
		frontRightMotor.setMode(mode);
		backLeftMotor.setMode(mode);
		backRightMotor.setMode(mode);
	}

	/*
	 * ====================================================================================================
	 * Driving "Helper" functions are below this line.
	 * These provide the high and low level methods that handle driving straight and turning.
	 * ====================================================================================================
	 */

	// **********  HIGH Level driving functions.  ********************

	/**
	 *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
	 *  Move will stop if either of these conditions occur:
	 *  1) Move gets to the desired position
	 *  2) Driver stops the OpMode running.
	 *
	 * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
	 * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
	 * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from the current robotHeading.
	 */
	public void driveStraight(double maxDriveSpeed, double distance, double heading) {

		// Ensure that the OpMode is still active
		if (opModeIsActive()) {

			// Determine new target position, and pass to motor controller
			int moveCounts = (int)(distance * COUNTS_PER_INCH);
			int leftTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
			int rightTarget = frontRightMotor.getCurrentPosition() + moveCounts;

			// Set Target FIRST, then turn on RUN_TO_POSITION
			frontLeftMotor.setTargetPosition(leftTarget);
			frontRightMotor.setTargetPosition(rightTarget);
			backLeftMotor.setTargetPosition(leftTarget);
			backRightMotor.setTargetPosition(rightTarget);

			setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// Set the required driving speed  (must be positive for RUN_TO_POSITION)
			// Start driving straight, and then enter the control loop
			maxDriveSpeed = Math.abs(maxDriveSpeed);
			moveRobot(maxDriveSpeed, 0);

			// keep looping while we are still active, and BOTH motors are running.
			while (opModeIsActive() &&
					(frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

				// Determine required steering to keep on heading
				turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

				// if driving in reverse, the motor correction also needs to be reversed
				if (distance < 0)
					turnSpeed *= -1.0;

				// Apply the turning correction to the current driving speed.
				moveRobot(driveSpeed, turnSpeed);

				// Display drive status for the driver.
				//sendTelemetry(true);
			}

			// Stop all motion & Turn off RUN_TO_POSITION
			moveRobot(0, 0);
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	/**
	 *  Spin on the central axis to point in a new direction.
	 *  <p>
	 *  Move will stop if either of these conditions occur:
	 *  <p>
	 *  1) Move gets to the heading (angle)
	 *  <p>
	 *  2) Driver stops the OpMode running.
	 *
	 * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
	 * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
	 *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *              If a relative angle is required, add/subtract from current heading.
	 */
	public void turnToHeading(double maxTurnSpeed, double heading) {

		// Run getSteeringCorrection() once to pre-calculate the current error
		getSteeringCorrection(heading, P_DRIVE_GAIN);

		// keep looping while we are still active, and not on heading.
		while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

			// Determine required steering to keep on heading
			turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

			// Clip the speed to the maximum permitted value.
			turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

			// Pivot in place by applying the turning correction
			moveRobot(0, turnSpeed);
		}

		// Stop all motion;
		moveRobot(0, 0);
	}

	/*/**
	 *  Obtain & hold a heading for a finite amount of time
	 *  <p>
	 *  Move will stop once the requested time has elapsed
	 *  <p>
	 *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
	 *
	 * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
	 * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
	 *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
	 *                   If a relative angle is required, add/subtract from current heading.
	 * @param holdTime   Length of time (in seconds) to hold the specified heading.
	 */
	/*/*public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

		ElapsedTime holdTimer = new ElapsedTime();
		holdTimer.reset();

		// keep looping while we have time remaining.
		while (opModeIsActive() && (holdTimer.time() < holdTime)) {
			// Determine required steering to keep on heading
			turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

			// Clip the speed to the maximum permitted value.
			turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

			// Pivot in place by applying the turning correction
			moveRobot(0, turnSpeed);

			// Display drive status for the driver.
			//sendTelemetry(false);
		}

		// Stop all motion;
		moveRobot(0, 0);
	}*/

	// **********  LOW Level driving functions.  ********************

	/**
	 * Use a Proportional Controller to determine how much steering correction is required.
	 *
	 * @param desiredHeading        The desired absolute heading (relative to last heading reset)
	 * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
	 * @return                      Turning power needed to get to required heading.
	 */
	public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
		// Determine the heading current error
		headingError = desiredHeading - getHeading();

		// Normalize the error to be within +/- 180 degrees
		while (headingError > 180)  headingError -= 360;
		while (headingError <= -180) headingError += 360;

		// Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
		return Range.clip(headingError * proportionalGain, -1, 1);
	}

	/**
	 * Take separate drive (fwd/rev) and turn (right/left) requests,
	 * combines them, and applies the appropriate speed commands to the left and right wheel motors.
	 * @param drive forward motor speed
	 * @param turn  clockwise turning motor speed.
	 */
	public void moveRobot(double drive, double turn) {
		driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
		turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

		double leftSpeed = drive - turn;
		double rightSpeed = drive + turn;

		// Scale speeds down if either one exceeds +/- 1.0;
		double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
		if (max > 1.0) {
			leftSpeed /= max;
			rightSpeed /= max;
		}

		frontLeftMotor.setPower(leftSpeed);
		frontRightMotor.setPower(rightSpeed);
		backLeftMotor.setPower(leftSpeed);
		backRightMotor.setPower(rightSpeed);
	}

	/*/**
	 *  Display the various control parameters while driving
	 *
	 * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
	 */
	/* /*
	private void sendTelemetry(boolean straight) {

		if (straight) {
			telemetry.addData("Motion", "Drive Straight");
			telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
			telemetry.addData("Actual Pos L:R",  "%7d:%7d:%7d:%7d", frontLeftMotor.getCurrentPosition(),
					frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(),
					backRightMotor.getCurrentPosition());
		} else {
			telemetry.addData("Motion", "Turning");
		}

		telemetry.addData("Heading Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
		telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
		telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
		telemetry.addData("Side", "%d", sideId);
		telemetry.update();
	}*/

	/**
	 * read the Robot heading directly from the IMU (in degrees)
	 */
	public double getHeading() {
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		return orientation.getYaw(AngleUnit.DEGREES);
	}
}