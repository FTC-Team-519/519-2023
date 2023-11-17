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

    public TestVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        telemetry.addData("Width: ", width);
        telemetry.addData("Height: ", height);
        telemetry.update();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int leftSide = 0;
        int center = 0;
        int rightSide = 0;
        double startTime = System.currentTimeMillis();

        for (int row = 0; row < 480; row++) {
            for (int col = 0; col < 640; col++) {
                double[] indexList = frame.get(row, col);
                double red = indexList[0];
                double green = indexList[1];
                double blue = indexList[2];
                if (col < 213) {
                    if (red > (green + blue)) {
                        leftSide++;
                    }
                }else if (col > 213 && col < 426){
                    if (red > (green + blue)) {
                        center++;
                    }
                }else if (col > 426){
                    if (red > (green + blue)) {
                        rightSide++;
                    }
                }
            }
        }
        double endTime = System.currentTimeMillis();
        if (leftSide > center && leftSide > rightSide) {
            telemetry.addData("Where the TSE is: ", "Left Side");
        }else if (center > leftSide && center > rightSide) {
            telemetry.addData("Where the TSE is: ", "Center");
        }else if (rightSide > leftSide && rightSide > center) {
            telemetry.addData("Where the TSE is: ", "Right Side");
        }else {
            telemetry.addData("Where the TSE is: ", "Not Found");
        }
        telemetry.addData("Times red seen on left: ", leftSide);
        telemetry.addData("Times red seen in center: ", center);
        telemetry.addData("Times red seen on right: ", rightSide);
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
}
