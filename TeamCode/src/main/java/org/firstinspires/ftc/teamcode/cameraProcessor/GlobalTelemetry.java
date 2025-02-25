package org.firstinspires.ftc.teamcode.cameraProcessor;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@Config
public class GlobalTelemetry {
    private static ArrayList<String[]> telemetryData = new ArrayList<String[]>();
    public static void addTelemetry(String caption, String value){
        telemetryData.add(new String[]{caption, value});
    }
    public static void addTelemetry(String caption, int value){
        telemetryData.add(new String[]{caption, String.valueOf(value)});
    }
    public static void addTelemetry(int caption, int value){
        telemetryData.add(new String[]{String.valueOf(caption), String.valueOf(value)});
    }
    public static void clearTelemetryData(){
        telemetryData = new ArrayList<String[]>();
    }
    public static ArrayList<String[]> getTelemetryData(){
        return telemetryData;
    }

}
