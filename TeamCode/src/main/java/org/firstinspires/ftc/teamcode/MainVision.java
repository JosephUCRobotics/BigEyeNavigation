/* Copyright (c) 2023 FIRST. All rights reserved.
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

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.cameraProcessor.BlobProcessor;
import org.firstinspires.ftc.teamcode.cameraProcessor.GlobalTelemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;


/**
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *n
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

public class MainVision {
    static int webcam;
    public MainVision() throws InterruptedException {
        initCameraProcessor();
        getCameraSetting();
        setManualExposure(CameraVariables.exposure, CameraVariables.gain);

        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) new CameraDisplaySource(), 15);


    }
    private void initCameraProcessor() {
        List myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        int Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();

        CameraVariables.blob = new BlobProcessor();


        // Create the AprilTag processor.
        CameraVariables.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1414.88, 1414.88, 709.08, 352.476)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        CameraVariables.aprilTag2 = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1414.88, 1414.88, 709.08, 352.476)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        CameraVariables.blob = new BlobProcessor();
        CameraReader cameraReader = new CameraReader(1);


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        //builder.setLiveViewContainerId(0);
        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }

        CameraVariables.webcam1 = CameraVariables.hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraVariables.webcam2 = CameraVariables.hardwareMap.get(WebcamName.class, "Webcam 2");

        builder.setCamera(CameraVariables.webcam1);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 960));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(CameraVariables.aprilTag2);
        builder.addProcessor(cameraReader);

        // Build the Vision Portal, using the above settings.
        builder.setLiveViewContainerId(Portal_1_View_ID);
        CameraVariables.visionPortal = builder.build();

        CameraVariables.visionPortal.setProcessorEnabled(CameraVariables.aprilTag2, false);
        CameraVariables.visionPortal.setProcessorEnabled(cameraReader, true);




        CameraReader cameraReader2 = new CameraReader(2);

        VisionPortal.Builder builder2 = new VisionPortal.Builder();

        builder2.setCamera(CameraVariables.webcam2);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder2.setCameraResolution(new Size(1280, 960));

        builder2.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder2.addProcessor(CameraVariables.aprilTag);
        builder2.addProcessor(cameraReader2);

        // Build the Vision Portal, using the above settings.
        builder2.setLiveViewContainerId(Portal_2_View_ID);
        CameraVariables.visionPortal2 = builder2.build();

        CameraVariables.visionPortal2.setProcessorEnabled(CameraVariables.aprilTag, false);
        CameraVariables.visionPortal2.setProcessorEnabled(cameraReader2, true);

    }   // end method initAprilTag()
    public static Vector<Double> getCameraPosWithAprilTag(AprilTagDetection detection) {
        double cx = detection.ftcPose.x;
        double cy = detection.ftcPose.y;
        // cx & cy are in inches

        double yaw = detection.ftcPose.yaw;
        // yaw is in digress

        double range = detection.ftcPose.range;
        // range is in inches

        double thetaA = 180;
        if (detection.id <= 6) thetaA = 0;
        // thetaA is in digress

        // the reason thees functions don't look right is because fieldPosition is rotated 90 digress from how we want it
        double ax = -detection.metadata.fieldPosition.get(1);
        double ay = detection.metadata.fieldPosition.get(0);
        // ax & ay are in inches

        double thetaCF = thetaA - yaw;
        // thetaCF is in digress

        if (cx == 0) {
            cx = 0.0000001;
        }

        double alpha = Math.atan(cy / cx);
        // alpha is in radians

        if (alpha < 0) {
            alpha += Math.PI;
        }

        double b = Math.toRadians(thetaCF) + alpha;
        // b is in radians

        double cxf = range * Math.cos(b);
        double cyf = range * Math.sin(b);
        // cxf & cyf are in inches


        //35.5, -70.5
        double by = ay - cyf;
        double bx = ax - cxf;
        // by & bx are in inches

        Vector<Double> output = new Vector<>();
        output.add(thetaCF);
        output.add(bx);
        output.add(by);
        output.add(ax);
        output.add(ay);
        output.add(thetaA);
        return output;
    }
    public static Vector<Double> getRobotPosFromCameraStream(double thetaCF, double by, double bx) {
        //
        //                                                          degrees
        double rx;
        double ry;
        double thetaCR;
        if (webcam == 2) {
            rx = 6;
            ry = 2;
            thetaCR = 131;
        } else {
            rx = 10;
            ry = 0;
            thetaCR = -33;
        }
//        double rx = -4;
//        double ry = 2;
//        double thetaCR = 45;
        // degrees
        double thetaRF = thetaCF - thetaCR;
        // degrees
        double range = Math.sqrt(Math.pow(ry, 2) + Math.pow(rx, 2));
        if (rx == 0) rx = 0.0000001;
        double alpha = Math.atan(ry / rx);
        // radians
        if (rx < 0) {
            if (ry > 0) {
                alpha += Math.PI;
            } else {
                alpha -= Math.PI;
            }
        }
        double b = Math.toRadians(thetaRF) + alpha;
        // radians
        double ryf = range * Math.sin(b);
        double rxf = range * Math.cos(b);
        double cy = by - ryf;
        double cx = bx - rxf;
        //thetaRF  += 90; // this is to change ware the zero is from up to right  ->

        Vector<Double> output = new Vector<>();
        output.add(thetaRF);
        output.add(cx);
        output.add(cy);
        return output;
    }
    private void getCameraSetting() throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (CameraVariables.visionPortal == null || CameraVariables.visionPortal2 == null) {
            return;
        }

        // Wait for the camera to be open
        if (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            GlobalTelemetry.clearTelemetryData();
            GlobalTelemetry.addTelemetry("Camera Status: ", "Waiting");
            while (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }
            GlobalTelemetry.clearTelemetryData();
            GlobalTelemetry.addTelemetry("Camera Status: ", "Ready");
        }

        ExposureControl exposureControl = CameraVariables.visionPortal.getCameraControl(ExposureControl.class);
        ExposureControl exposureControl2 = CameraVariables.visionPortal.getCameraControl(ExposureControl.class);
        CameraVariables.minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        CameraVariables.maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = CameraVariables.visionPortal.getCameraControl(GainControl.class);
        CameraVariables.minGain = gainControl.getMinGain();
        CameraVariables.maxGain = gainControl.getMaxGain();
    }
    protected static boolean setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Ensure Vision Portal has been setup.
        if (CameraVariables.visionPortal == null || CameraVariables.visionPortal2 == null) {
            return false;
        }

        // Wait for the camera to be open
        if (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            GlobalTelemetry.clearTelemetryData();
            GlobalTelemetry.addTelemetry("Camera Status: ", "Waiting");
            while (CameraVariables.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING || CameraVariables.visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }
            GlobalTelemetry.clearTelemetryData();
            GlobalTelemetry.addTelemetry("Camera Status: ", "Ready");
        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = CameraVariables.visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        ExposureControl exposureControl2 = CameraVariables.visionPortal2.getCameraControl(ExposureControl.class);
        if (exposureControl2.getMode() != ExposureControl.Mode.Manual) {
            exposureControl2.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        exposureControl2.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);

        // Set Gain.
        GainControl gainControl = CameraVariables.visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);

        GainControl gainControl2 = CameraVariables.visionPortal2.getCameraControl(GainControl.class);
        gainControl2.setGain(gain);
        Thread.sleep(20);
        return (true);

    }
    public static class CameraDisplaySource implements CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            if (CameraVariables.lastBitmap != null){
                lastFrame.set(CameraVariables.lastBitmap);
                continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
            }
        }
    }
    public static class CameraReader implements VisionProcessor {
        int id;
        CameraReader(int idIn){
            id = idIn;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if(id == 1){
                CameraVariables.cameraStream1 = frame;
            } else {
                CameraVariables.cameraStream2 = frame;
            }

            if (CameraVariables.cameraStream1 != null && CameraVariables.cameraStream2 != null && id == 1){
                // This rotates the cameras
                Mat rotatedCameraStream1 = new Mat();
                Mat rotatedCameraStream2 = new Mat();
                Core.rotate(CameraVariables.cameraStream1, rotatedCameraStream1,Core.ROTATE_90_CLOCKWISE);
                Core.rotate(CameraVariables.cameraStream2, rotatedCameraStream2,Core.ROTATE_90_CLOCKWISE);

                // process frames separately hear
                Mat[] separatelyProcessedMats = MainVision.processMatsSeparately(rotatedCameraStream1, rotatedCameraStream2);

                // This joins the cameras in to one mat
                Mat dst = new Mat();
                List<Mat> src = Arrays.asList(separatelyProcessedMats[0], separatelyProcessedMats[1]);
                Core.hconcat(src, dst);

                // process frames as one hear

                Mat ProcessedMat = MainVision.processMatsAsOne(dst);

                // convert the mat to a bitmap before displaying it on the ftc dashboard
                Bitmap b = Bitmap.createBitmap(ProcessedMat.width(), ProcessedMat.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(ProcessedMat, b);

                CameraVariables.lastBitmap = b;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }
    }
    protected static Mat[] processMatsSeparately(Mat leftMat, Mat rightMat){

        return new Mat[]{leftMat,rightMat};
    }
    protected static Mat processMatsAsOne(Mat combinedMat){
        combinedMat = CameraVariables.blob.processFrame(combinedMat);
        return combinedMat;
    }
}

