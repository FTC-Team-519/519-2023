package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class TestVisionProcessor implements VisionProcessor {
    private final Telemetry telemetry;
    private boolean isRed;
    private boolean center;
    private boolean left;
    private boolean right;

    public TestVisionProcessor(Telemetry telemetry, boolean lookingForRed) {
        isRed = lookingForRed;
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        left = false;
        right = false;
        center = false;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int leftSideTimesSeen = 0;
        int centerTimesSeen = 0;
        int rightSideTimesSeen = 0;
        double startTime = System.currentTimeMillis();

        for (int row = 0; row < 480; row++) {
            for (int col = 0; col < 640; col++) {
                double[] indexList = frame.get(row, col);
                double red = indexList[0];
                double green = indexList[1];
                double blue = indexList[2];
                if (col < 213) {
                    if (isRed && red > (green + blue)) {
                        leftSideTimesSeen++;
                    } else if (!isRed && blue > (green + red)){
                        leftSideTimesSeen++;
                    }
                }else if (col > 213 && col < 426){
                    if (isRed && red > (green + blue)) {
                        centerTimesSeen++;
                    }else if (!isRed && blue > (green + red)) {
                        centerTimesSeen++;
                    }
                }else if (col > 426){
                    if (isRed && red > (green + blue)) {
                        rightSideTimesSeen++;
                    }else if (!isRed && blue > (green + red)) {
                        rightSideTimesSeen++;
                    }
                }
            }
        }
        double endTime = System.currentTimeMillis();
        if (leftSideTimesSeen > centerTimesSeen && leftSideTimesSeen > rightSideTimesSeen) {
            left = true;
            telemetry.addData("Where the TSE is: ", "Left Side");
        }else if (centerTimesSeen > leftSideTimesSeen && centerTimesSeen > rightSideTimesSeen) {
            center = true;
            telemetry.addData("Where the TSE is: ", "Center");
        }else if (rightSideTimesSeen > leftSideTimesSeen && rightSideTimesSeen > centerTimesSeen) {
            right = true;
            telemetry.addData("Where the TSE is: ", "Right Side");
        }else {
            left = false;
            right = false;
            center = false;
            telemetry.addData("Where the TSE is: ", "Not Found");
        }
        telemetry.addData("Times red seen on left: ", leftSideTimesSeen);
        telemetry.addData("Times red seen in center: ", centerTimesSeen);
        telemetry.addData("Times red seen on right: ", rightSideTimesSeen);
        telemetry.addData("Width: ", frame.width());
        telemetry.addData("Height", frame.height());
        telemetry.addData("Time Elapsed: ", (endTime - startTime));
//        telemetry.update();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint test = new Paint();
//        test.setColor(Color.RED);
//        canvas.drawLine(416, 0, 416, 512, test);
//        canvas.drawLine(832, 0, 832, 512, test);
//        telemetry.addData("Canvas Height: ", canvas.getHeight());
//        telemetry.addData("Canvas Width: ", canvas.getWidth());
    }

    public boolean getLeft() {
        return left;
    }

    public boolean getCenter() {
        return center;
    }

    public boolean getRight() {
        return right;
    }
}