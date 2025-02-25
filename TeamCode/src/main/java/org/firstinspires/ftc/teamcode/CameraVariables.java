package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.cameraProcessor.BlobProcessor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

@Config

public class CameraVariables {
    public static int minGain;
    public static int maxGain;
    protected static Bitmap lastBitmap;
    protected static ExposureControl exposureControl;
    public static int gain = 50;
    public static int exposure = 5;
    protected static int minExposure;
    protected static int maxExposure;

    // TestProcessor
    public static Mat cameraStream1;
    public static Mat cameraStream2;
    // BlobFinder
    protected static  VisionPortal visionPortal; // Used to manage the video source.
    protected static  VisionPortal visionPortal2; // Used to manage the video source.

    protected static  AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    protected static  AprilTagProcessor aprilTag2;
    protected static BlobProcessor blob;
    protected static  WebcamName webcam1, webcam2;

    // ManeVision
    public static HardwareMap hardwareMap;

    // BlobProcessor
    public static int blurLevel = 20;
    public static int lowerScalar1 = 10;
    public static int lowerScalar2 = 100;
    public static int lowerScalar3 = 135;
    public static int upperScalar1 = 115;
    public static int upperScalar2 = 125;
    public static int upperScalar3 = 200;
    public static int erodeLevel = 20;
    public static int dilateLevel = 50;
    public static int cameraWidth = 1280;
    public static int cameraHeight = 720;
    public static int distanceX;

    public static int wheelSpeed = 2;
}
