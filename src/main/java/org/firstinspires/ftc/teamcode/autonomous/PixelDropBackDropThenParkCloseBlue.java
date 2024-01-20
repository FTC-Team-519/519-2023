package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Close Side Place On Backdrop")
public class PixelDropBackDropThenParkCloseBlue extends PixelDropNoMoveAfterBlue{
    private DriveToAprilTag tagDetection = new DriveToAprilTag(telemetry);
    private static final int LEFT_BLUE_TAG_ID = 1;
    private static final int CENTER_BLUE_TAG_ID = 2;
    private static final int RIGHT_BLUE_TAG_ID = 3;

    private enum ParkingSteps {
        GO_FORWARD_INCHS,
        STRAFE_RIGHT, // Might need to make this be turn left
        TURN_LEFT,
        GO_FORWARD,
        FIND_AND_GO_TO_TAG,
        STRAFE_RIGHT_FOR_BACKDROP,
        SCORE_ON_BACKDROP,
        GO_FORWARD_TO_BACKDROP,
        RELEASE_PIXEL_FROM_PLACER,
        PARK,
        DONE
    }

    private ParkingSteps parkingSteps = ParkingSteps.GO_FORWARD_INCHS;
    @Override
    public void init() {
        super.init();
        tagDetection.init(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, aprilTagProcessor);
        parkingSteps = ParkingSteps.GO_FORWARD_INCHS;
        teamScoringElementFinder.setToCloseSide();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        if (!isDone()) {
            super.loop();
        } else {
            switch (parkingSteps) {
                case GO_FORWARD_INCHS:
                    boolean done = driveDistanceInches(0.5, 4);
                    if (runtime.milliseconds() > 3000 || done) {
                        runtime.reset();
                        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                                parkingSteps = ParkingSteps.STRAFE_RIGHT;
                        parkingSteps = ParkingSteps.TURN_LEFT;
                        done = false;
                    }
                    break;
                case TURN_LEFT:
                    done = turnLeft(0.5, 65);
                    if (runtime.milliseconds() > 4000 || done) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.GO_FORWARD;
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        done = false;
                    }
                    break;
                case GO_FORWARD:
                    done = driveDistanceInches(0.5, 24);
                    if (runtime.milliseconds() > 5000 || done) {
                        runtime.reset();
                        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        parkingSteps = ParkingSteps.FIND_AND_GO_TO_TAG;
                        done = false;
                    }
                    break;
                case FIND_AND_GO_TO_TAG:
                    boolean isFound = false;
                    if (positionOfTheTSE == PositionOfTSE.LEFT) {
                        isFound = tagDetection.moveToTargetId(CENTER_BLUE_TAG_ID);
                    }else if (positionOfTheTSE == PositionOfTSE.CENTER) {
                        isFound = tagDetection.moveToTargetId(RIGHT_BLUE_TAG_ID);
                    }else if (positionOfTheTSE == PositionOfTSE.RIGHT) {
                        isFound = tagDetection.moveToTargetId(RIGHT_BLUE_TAG_ID);
                    }
                    if (isFound && positionOfTheTSE == PositionOfTSE.RIGHT) {
                        runtime.reset();
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        parkingSteps = ParkingSteps.STRAFE_RIGHT_FOR_BACKDROP;
                    }else if (isFound) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.GO_FORWARD_TO_BACKDROP;
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    break;
                case STRAFE_RIGHT_FOR_BACKDROP:
                    done = strafeRight((int)(6.5 * COUNTS_PER_INCH), 0.35);
                    if (runtime.milliseconds() > 3000 || done) {
                        parkingSteps = ParkingSteps.GO_FORWARD_TO_BACKDROP;
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        runtime.reset();
                        done = false;
                    }
                    break;
                case GO_FORWARD_TO_BACKDROP:
                    done = driveDistanceInches(0.5, 6);
                    if (runtime.milliseconds() > 1500 || done) {
                        parkingSteps = ParkingSteps.SCORE_ON_BACKDROP;
                        runtime.reset();
                        done = false;
                    }
                    break;

                case SCORE_ON_BACKDROP:
                    autoBackdrop.setPosition(OPEN_VALUE_FOR_PIXEL_BACKDROPPER);
                    if (runtime.milliseconds() > 500) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.RELEASE_PIXEL_FROM_PLACER;
                    }
                    break;
                case RELEASE_PIXEL_FROM_PLACER:
                    autoBackdrop.setPosition(CLOSED_VALUE_FOR_PIXEL_BACKDROPPER);
                    if (runtime.milliseconds() > 500) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.DONE;
                    }
                    break;
            }
        }
        telemetry.addData("Step On", parkingSteps);
    }

    protected boolean ParkingDone() {
        return parkingSteps == ParkingSteps.DONE;
    }
}

