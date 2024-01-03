package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.autonomous.TSEVisionProcessor;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;


public class BaseAutoTestToHome extends OpMode {
    protected static final double MAX_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.97;
    protected static final double MIN_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.31;
    protected static final double CLOSED_VALUE_FOR_PIXEL_DROPPER = 0.34;
    protected static final double OPEN_VALUE_FOR_PIXEL_DROPPER = 0.40;

    protected static final double CLOSED_VALUE_FOR_PIXEL_BACKDROPPER = 0.45;
    protected static final double OPEN_VALUE_FOR_PIXEL_BACKDROPPER = 0.2;
    protected static final double MIN_VALUE_FOR_WRIST_SERVO = 0.27;
    protected static final double MAX_VALUE_FOR_WRIST_SERVO = 0.85;

    protected AprilTagProcessor aprilTagProcessor;
    protected TSEVisionProcessor teamScoringElementFinder;
    protected VisionPortal portal;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected Servo autoPixelServo = null;
    protected Servo autoBackdrop = null;
    protected ColorSensor pixelDropperColorSensor;

    protected IMU imu = null;

    boolean orientationIsValid = false;
    //protected Orientation angles;
    double yaw;
    double pitch;
    double roll;

    private boolean goingCenter = false;
    private boolean goingLeft = false;
    private boolean goingRight = false;

    protected boolean onRedTeam = true;

    protected PositionOfTSE positionOfTheTSE = PositionOfTSE.RIGHT;

    protected ElapsedTime runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 1425.1  ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    private static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected double  driveSpeed    = 0.25;

    protected double angleOffset;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;

    protected enum PositionOfTSE {
        LEFT,
        CENTER,
        RIGHT
    }

    private enum PixelDropStep {
        START,
        CHECK_TARGET,
        RUN_TIME_RESET_FOR_TURN,
        TURN_TO_SPIKE,
        RUN_TIME_RESET_FOR_DRIVE_TO_SPIKE,
        DRIVE_TO_SPIKE,
        DROP_PIXEL,
        FINAL_RESET,
        DRIVE_BACK_TO_HOME,
        DONE
    }

    private PixelDropStep step = PixelDropStep.START;

    private double distance_traveled_forward = 0;
    private double angle_turned = 0;
    private double distance_traveled_to_pixel = 0;


    @Override
    public void init() {
        autoPixelServo = hardwareMap.get(Servo.class, "pixelDropperServo");
        autoBackdrop = hardwareMap.get(Servo.class, "autoBackdrop");
        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        pixelDropperColorSensor = hardwareMap.get(ColorSensor.class, "pixelColorSensor");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imuExpan");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu.initialize(parameters);
        orientationIsValid = false;
        yaw = 0.0d;
        pitch = 0.0d;
        roll = 0.0d;

        try {
            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();

            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }


        autoPixelServo.setPosition(CLOSED_VALUE_FOR_PIXEL_DROPPER);
        autoBackdrop.setPosition(CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);

        angleOffset = 0;

        moveWrist(MIN_VALUE_FOR_WRIST_SERVO);

        initAprilTag();
    }

    @Override
    public void init_loop() {
        if (teamScoringElementFinder.isCenter()) {
            positionOfTheTSE = PositionOfTSE.CENTER;
        } else if (teamScoringElementFinder.isRight()) {
            positionOfTheTSE = PositionOfTSE.RIGHT;
        } else if (teamScoringElementFinder.isLeft()) {
            positionOfTheTSE = PositionOfTSE.LEFT;
        }
    }

    @Override
    public void start() {
        teamScoringElementFinder.stop();
        runtime.reset();
    }

    @Override
    public void loop() {
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        yaw = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
        pitch = yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
        roll = yawPitchRollAngles.getRoll(AngleUnit.DEGREES);
        switch (positionOfTheTSE) {
            case LEFT: // Going to the left
                switch (step) {
                    case START:
//                        driveDistanceTime(driveSpeed, 5000, runtime);
                        driveDistanceInches(0.25, 12);
                        step = PixelDropStep.CHECK_TARGET;
                        break;
                    case CHECK_TARGET:
                        if (atTargetPosition()) {
                            step = PixelDropStep.RUN_TIME_RESET_FOR_TURN;
                            distance_traveled_forward = leftFrontDrive.getCurrentPosition();
                        }
                        break;
                    case RUN_TIME_RESET_FOR_TURN:
//                        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        step = PixelDropStep.TURN_TO_SPIKE;
                        break;
                    case TURN_TO_SPIKE:
                        boolean leftTurnIsDone = turnLeft(0.25, 45);
                        if (runtime.seconds() > 4.0 || leftTurnIsDone) {
                            step = PixelDropStep.RUN_TIME_RESET_FOR_DRIVE_TO_SPIKE;
                        }
                        break;
                    case RUN_TIME_RESET_FOR_DRIVE_TO_SPIKE:
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        runtime.reset();
                        step = PixelDropStep.DRIVE_TO_SPIKE;
                        break;
                    case DRIVE_TO_SPIKE:
                        setAllDrivePower(0.25);
                        if ((onRedTeam && seeingRed()) || (!onRedTeam && seeingBlue())) {
                            setAllDrivePower(0.0);
                            step = PixelDropStep.DROP_PIXEL;
                            distance_traveled_to_pixel = leftFrontDrive.getCurrentPosition();
                        }
                        break;
                    case DROP_PIXEL:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        step = PixelDropStep.FINAL_RESET;
                        break;
                    case DRIVE_BACK_TO_HOME:
                        break;



                    case FINAL_RESET:
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        runtime.reset();
                        step = PixelDropStep.DONE;
                        break;
                }
                break;


            case CENTER: // Going to Center
                switch (step) {
                    case START:
//                        driveDistanceInches(driveSpeed, 36);
                        setAllDrivePower(0.25);
                        step = PixelDropStep.DRIVE_TO_SPIKE;
                        break;
                    case DRIVE_TO_SPIKE:
                        if ((onRedTeam && seeingRed()) || (!onRedTeam && seeingBlue())) {
                            setAllDrivePower(0.0);
                            step=PixelDropStep.FINAL_RESET;
                        }
                        break;
                    case DROP_PIXEL:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        step = PixelDropStep.FINAL_RESET;
                    case FINAL_RESET:
                        runtime.reset();
                        step=PixelDropStep.DONE;
                        break;
                }
                break;


            case RIGHT: // Going to the right
                switch (step) {
                    case START:
                        driveDistanceInches(0.25, 12);
                        step=PixelDropStep.CHECK_TARGET;
                        break;
                    case CHECK_TARGET:
                        if (atTargetPosition()) {
                            step=PixelDropStep.RUN_TIME_RESET_FOR_TURN;
                        }
                        break;
                    case RUN_TIME_RESET_FOR_TURN:
                        runtime.reset();
                        step=PixelDropStep.TURN_TO_SPIKE;
                        break;
                    case TURN_TO_SPIKE:
                        boolean rightTurnIsDone = turnRight(0.25, 45);
                        if (runtime.seconds() > 4.0 || rightTurnIsDone) {
                            step=PixelDropStep.RUN_TIME_RESET_FOR_DRIVE_TO_SPIKE;
                        }
                        break;
                    case RUN_TIME_RESET_FOR_DRIVE_TO_SPIKE:
                        runtime.reset();
                        step=PixelDropStep.DRIVE_TO_SPIKE;
                        break;
                    case DRIVE_TO_SPIKE:
                        setAllDrivePower(0.25);

                        if ((onRedTeam && seeingRed()) || (!onRedTeam && seeingBlue())) {
                            setAllDrivePower(0.0);
                            step=PixelDropStep.DROP_PIXEL;
                        }
                        break;
                    case DROP_PIXEL:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        step=PixelDropStep.FINAL_RESET;
                        break;
                    case FINAL_RESET:
                        runtime.reset();
                        step = PixelDropStep.DONE;
                        break;

                }
        }
        telemetry.addData("Step:", step);
        telemetry.addData("Heading", yaw);
        telemetry.addData("Pitch", pitch);
        telemetry.addData("Roll", roll);
        telemetry.addData("TSE position:", "%d", positionOfTheTSE);

        displayTargetAndActualPosition(leftFrontDrive);
        displayTargetAndActualPosition(leftBackDrive);
        displayTargetAndActualPosition(rightFrontDrive);
        displayTargetAndActualPosition(rightBackDrive);

        telemetry.update();
    }

    @Override
    public void stop() {}

//    protected void driveDistanceTime(double speed, double desiredTimeMiliseconds, ElapsedTime timeElapsed) {
//        if (timeElapsed.milliseconds() >= desiredTimeMiliseconds) {
//            step++;
//            setAllDrivePower(0.0);
//        }else {
//            setAllDrivePower(speed);
//        }
//    }

    protected void driveDistanceInches(double speed, double distanceInches) {
//        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setTargetPosition((int)(distanceInches * COUNTS_PER_INCH));
        setAllDrivePower(speed);
//        if(senseRed && !senseBlue) {
//            while (!atTargetPosition() && !seeingRed()) {
//                setAllDrivePower(speed);
//            }
//        }
//        else if (senseBlue && !senseRed) {
//            while (!atTargetPosition() && !seeingBlue()) {
//                setAllDrivePower(speed);
//            }
//        }
//        else if (senseBlue && senseRed) {
//            while (!atTargetPosition() && !seeingBlue() && !seeingRed()) {
//                setAllDrivePower(speed);
//            }
//        }
//        else {
//            while(!atTargetPosition()) {
//                setAllDrivePower(speed);
//            }
//        }
//        setAllDrivePower(0);
    }

    protected boolean atTargetPosition() {
        return (
                leftFrontDrive.getCurrentPosition() >= leftFrontDrive.getTargetPosition() &&
                        leftBackDrive.getCurrentPosition() >= leftBackDrive.getTargetPosition() &&
                        rightFrontDrive.getCurrentPosition() >= rightFrontDrive.getTargetPosition() &&
                        rightBackDrive.getCurrentPosition() >= rightBackDrive.getTargetPosition()
        );
    }
    protected void setTargetPosition(int position) {
        leftFrontDrive.setTargetPosition(position);
        leftBackDrive.setTargetPosition(position);
        rightFrontDrive.setTargetPosition(position);
        rightBackDrive.setTargetPosition(position);
    }

    private void displayTargetAndActualPosition(DcMotor motor) {
        telemetry.addData(motor.getDeviceName(), "Target: %d, Actual: %d", motor.getTargetPosition(), motor.getCurrentPosition());
    }

    protected boolean turnRight(double speed, double degree) {
        double targetDegree = angleOffset - degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (yaw >= targetDegree) {
            setLeftDrivesPower(speed);
            setRightDrivesPower(-speed);
        } else {
            setAllDrivePower(0.0);
            return true;
        }
        return false;
    }

    protected boolean turnLeft(double speed, double degree) {
//        Returns if done
        double targetDegree = angleOffset + degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (yaw <= targetDegree) {
            setLeftDrivesPower(-speed);
            setRightDrivesPower(speed);
        } else {
            setAllDrivePower(0.0);
            return true;
        }
        return false;
    }

    protected void setAllDrivePower(double power) {
        setLeftDrivesPower(power);
        setRightDrivesPower(power);
    }

    protected void setLeftDrivesPower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    protected void setRightDrivesPower(double power) {
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    protected boolean seeingBlue() {
        return pixelDropperColorSensor.blue() > pixelDropperColorSensor.red() && pixelDropperColorSensor.blue() > pixelDropperColorSensor.green();
    }

    protected boolean seeingGrey() {
        return pixelDropperColorSensor.green() > pixelDropperColorSensor.red() && pixelDropperColorSensor.green() > pixelDropperColorSensor.blue();
    }

    protected boolean seeingRed() {
        return pixelDropperColorSensor.red() > pixelDropperColorSensor.green() && pixelDropperColorSensor.red() > pixelDropperColorSensor.blue();
    }

    protected void strafeLeft(int distanceInches, double power){
        leftFrontDrive.setTargetPosition(-distanceInches);
        leftBackDrive.setTargetPosition(distanceInches);
        rightFrontDrive.setTargetPosition(distanceInches);
        rightBackDrive.setTargetPosition(-distanceInches);
        setAllDrivePower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void strafeRight(int value, double power){
        leftFrontDrive.setTargetPosition(value);
        leftBackDrive.setTargetPosition(-value);
        rightFrontDrive.setTargetPosition(-value);
        rightBackDrive.setTargetPosition(value);
        setAllDrivePower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void moveWrist(double position) {
        wristServoDroneSide.setPosition(position);
        wristServoControlHubSide.setPosition(position);
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            portal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
//        } else {
//            portal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, aprilTagProcessor);
//        }
    }

    public boolean isDone() {
        return step == PixelDropStep.DONE;
    }
}
