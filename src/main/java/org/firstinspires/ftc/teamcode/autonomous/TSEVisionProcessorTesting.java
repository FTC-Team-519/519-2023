package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Mat;

public class TSEVisionProcessorTesting implements org.firstinspires.ftc.vision.VisionProcessor {
    private final Telemetry telemetry;
    private boolean isRed;
    private boolean farSide = false;
    private boolean center;
    private boolean left;
    private boolean right;
    private boolean stopped = false;

    public enum SideTSEIsOn {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    private SideTSEIsOn sideTSEIsOn = SideTSEIsOn.NOT_FOUND;

    public TSEVisionProcessorTesting(Telemetry telemetry, boolean lookingForRed) {
        isRed = lookingForRed;
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        sideTSEIsOn = SideTSEIsOn.NOT_FOUND;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int leftSideTimesSeen = 0;
        int centerTimesSeen = 0;
        int rightSideTimesSeen = 0;
        double startTime = System.currentTimeMillis();

        if (!stopped) {
            for (int col = 0; col < 640; col++) {
                int numRed = 0;
                int numBlue = 0;
                for (int row = 240; row < 480; row++) {
                    double[] indexList = frame.get(row, col);
                    double red = indexList[0];
                    double green = indexList[1];
                    double blue = indexList[2];

                    if (isRed && red > (green + blue)) {
                        numRed++;
                    } else if (!isRed && blue > (green + red)) {
                        numBlue++;
                    }
                }
                if (col < 213 && (numRed > 20 || numBlue > 20)) {
                    leftSideTimesSeen++;
                } else if ((col > 213 && col < 426) && (numRed > 20 || numBlue > 20)) {
                    centerTimesSeen++;
                } else if (col > 426 && (numRed > 20 || numBlue > 20)) {
                    rightSideTimesSeen++;
                }
            }

            if (centerTimesSeen == 0 && rightSideTimesSeen == 0) {
                sideTSEIsOn = SideTSEIsOn.LEFT;
//                telemetry.addData("Where the TSE is: ", "Left Side");
            } else if (leftSideTimesSeen == 0 && rightSideTimesSeen == 0) {
                sideTSEIsOn = SideTSEIsOn.CENTER;
//                telemetry.addData("Where the TSE is: ", "Center");
            } else if (leftSideTimesSeen == 0 && centerTimesSeen == 0) {
                sideTSEIsOn = SideTSEIsOn.RIGHT;
//                telemetry.addData("Where the TSE is: ", "Right Side");
            }

            if (!farSide) {
                if (isRed) {
                    if (centerTimesSeen == 0 && rightSideTimesSeen == 0) {
                        sideTSEIsOn = SideTSEIsOn.LEFT;
//                        telemetry.addData("Where the TSE is: ", "Left Side");
                    }
                }else {
                    if (leftSideTimesSeen == 0 && centerTimesSeen == 0) {
                        sideTSEIsOn = SideTSEIsOn.RIGHT;
//                        telemetry.addData("Where the TSE is: ", "Right Side");
                    }
                }
            } else {
                if (isRed) {
                    if (leftSideTimesSeen == 0 && centerTimesSeen == 0) {
                        sideTSEIsOn = SideTSEIsOn.RIGHT;
//                        telemetry.addData("Where the TSE is: ", "Right Side");
                    }
                } else {
                    if (centerTimesSeen == 0 && rightSideTimesSeen == 0) {
                        sideTSEIsOn = SideTSEIsOn.LEFT;
//                        telemetry.addData("Where the TSE is: ", "Left Side");
                    }
                }
            }

//            if (leftSideTimesSeen > centerTimesSeen && leftSideTimesSeen > rightSideTimesSeen) {
//                left = true;
//                right = false;
//                center = false;
//                telemetry.addData("Where the TSE is: ", "Left Side");
//            } else if (centerTimesSeen > leftSideTimesSeen && centerTimesSeen > rightSideTimesSeen) {
//                center = true;
//                left = false;
//                right = false;
//                telemetry.addData("Where the TSE is: ", "Center");
//            } else if (rightSideTimesSeen > leftSideTimesSeen && rightSideTimesSeen > centerTimesSeen) {
//                right = true;
//                center = false;
//                left = false;
//                telemetry.addData("Where the TSE is: ", "Right Side");
//            } else {
//                left = false;
//                right = false;
//                center = false;
//                telemetry.addData("Where the TSE is: ", "Not Found");
//            }
        }
        double endTime = System.currentTimeMillis();
        telemetry.addData("Where the TSE is: ", sideTSEIsOn);
        telemetry.addData("Times red seen on left: ", leftSideTimesSeen);
        telemetry.addData("Times red seen in center: ", centerTimesSeen);
        telemetry.addData("Times red seen on right: ", rightSideTimesSeen);
//        telemetry.addData("Width: ", frame.width());
//        telemetry.addData("Height", frame.height());
        telemetry.addData("Time Elapsed: ", (endTime - startTime));
        telemetry.update();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    public boolean isLeft() {
        return sideTSEIsOn == SideTSEIsOn.LEFT;
    }

    public boolean isCenter() {
        return sideTSEIsOn == SideTSEIsOn.CENTER;
    }

    public boolean isRight() {
        return sideTSEIsOn == SideTSEIsOn.RIGHT;
    }

    public void setToFarSide() {
        farSide = true;
    }

    public void setToCloseSide() {
        farSide = false;
    }

    public void stop() {
        stopped = true;
    }
}
