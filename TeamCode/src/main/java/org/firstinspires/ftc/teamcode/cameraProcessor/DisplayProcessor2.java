package org.firstinspires.ftc.teamcode.cameraProcessor;

import android.graphics.Canvas;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CameraVariables;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

public class DisplayProcessor2 implements VisionProcessor {
    //  reference EOCV-Sim:
//  https://deltacv.gitbook.io/eocv-sim/vision-portal/introduction-to-visionportal/creating-and-running-a-visionprocessor
    int ID;
    Mat displayed;
    int pass = 0;
    public DisplayProcessor2(int id)  {
        ID = id;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration){
        // Initialization code here
        // Executed before the first call to processFrame
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if(ID == 1){
            CameraVariables.cameraStream1 = frame;
        } else {
            CameraVariables.cameraStream2 = frame;
        }

        if (pass > 10){
//            Mat dstTop = new Mat();
//            List<Mat> src = Arrays.asList(CameraVariables.cameraStream2, CameraVariables.cameraStream1);
//            Core.hconcat(src, dstTop);
//
//            //Mat dstBot = new Mat();
////            src = Arrays.asList(new Mat(CameraVariables.test1.rows(), CameraVariables.test1.cols(), CameraVariables.test1.type()),
////                    new Mat(CameraVariables.test1.rows(), CameraVariables.test1.cols(), CameraVariables.test1.type()));
////            Core.hconcat(src, dstBot);
//
//            Mat dst = new Mat();
//            List<Mat> src2 = Arrays.asList(dstTop, dstTop);
//            Core.vconcat(src2, dst);
            Mat dst = new Mat();
            List<Mat> src = Arrays.asList(CameraVariables.cameraStream2, CameraVariables.cameraStream1);
            Core.vconcat(src, dst);
            Mat dst2 = new Mat();
            Core.rotate(dst, dst2,Core.ROTATE_90_CLOCKWISE);
//            Mat dst3 = new Mat();
//            List<Mat> src2 = Arrays.asList(dst2, new Mat(new Size(1440,160),dst2.type()));
//            Core.vconcat(src2, dst3);
            displayed = new Mat(frame.size(),dst2.type());
        }
        if (pass < 20){
            pass += 1;
        }
        // Process the frame here
        // Executed every time a new frame is dispatched
        // if you change the mat "frame", that change will display
//        Mat newM = applyColorFilter(blur(frame));
//        Mat f2 = frame;

        // Create a mask (binary image) with the same size as the image
        return displayed;// Return the image that will be displayed in the viewport
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Paint Black over the default camera stream
//        if (displayed != null) {
//            Rect rect = new Rect(0, 0, 1280, 720);
//
//            Paint rectPaint = new Paint();
//            rectPaint.setColor(Color.BLACK);
//            rectPaint.setStyle(Paint.Style.STROKE);
//            rectPaint.setStrokeWidth(CameraVariables.cameraWidth);
//
//            canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
//
//
////            Bitmap bitmap = null;
////            Mat tmp = new Mat (displayed.cols(), displayed.rows(), CvType.CV_8U, new Scalar(4));
////            try {
////                //Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
////                Imgproc.cvtColor(displayed, tmp, Imgproc.COLOR_GRAY2RGBA, 4);
////                bitmap = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);
////                Utils.matToBitmap(tmp, bitmap);
////            }
////            catch (CvException e){
////                Log.d("Exception",e.getMessage());}
//
//            // Convert the OpenCV Mat to a Bitmap
//            Bitmap bitmap = Bitmap.createBitmap(displayed.cols(), displayed.rows(), Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(displayed, bitmap);
//
//            // Create a Paint object for drawing
//            Paint paint = new Paint();
//            paint.setAntiAlias(true);
//
//            Rect destinationRect = new Rect(0, 0, CameraVariables.cameraWidth, CameraVariables.cameraHeight);
//            // Draw the Bitmap on the Canvas
//            canvas.drawBitmap(bitmap, null, destinationRect, paint);
        //}

    }
    private Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.left * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.top * scaleBmpPxToCanvasPx);
        int right = Math.round(rect.right * scaleBmpPxToCanvasPx);
        int bottom =Math.round(rect.bottom * scaleBmpPxToCanvasPx);

        return new Rect(left, top, right, bottom);
    }

}
