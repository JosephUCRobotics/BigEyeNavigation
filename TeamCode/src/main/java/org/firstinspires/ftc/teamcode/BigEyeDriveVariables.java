package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BigEyeDriveVariables {

    // Positive Y is forward.  Positive X is Right.
    // The origin is the center of rotation.

    // Front
    //          {wheel 1, wheel 2, wheel 3, wheel4}
    //          {front, left, back, right}

    // If the number of wheels change, change wheelXMovement, wheelYMovement, wheelXFromOrigin, wheelYFromOrigin
    //      wheelTixPerRotation, wheelTuner, startingWheelPositions, newArray(BigEyeDrive)
    // There should be one calm for each wheel
    public static double[] wheelXMovement = {90*Math.PI/25.4, 0, -90*Math.PI/25.4, 0};
    public static double[] wheelYMovement = {0, 90*Math.PI/25.4, 0, 90*Math.PI/25.4};
    public static double[] wheelXFromOrigin = {-7.75, -7.75, 7.75 ,7.75};
    public static double[] wheelYFromOrigin = {7.75, -7.75, -7.75 ,7.75};
    public static double[] wheelTixPerRotation = {28*20, 28*20, 28*20, 28*20};
    public static double[] wheelTuner = {1, 1, 1, 1};

    public static boolean trackPos = false;
    public static double[] startingContinents = {0, 0, 0};
//    // Left
//    public static double wheel2XMovement = 90/25.4;
//    public static double wheel2YMovement = 0;
//    public static double wheel2XFromOrigin = 0;
//    public static double wheel2YFromOrigin = 0;
//    public static double wheel2TixPerRotation = 28*20;
//    public static double wheel2Tuner = 1;
//    // Back
//    public static double wheel3XMovement = 90/25.4;
//    public static double wheel3YMovement = 0;
//    public static double wheel3XFromOrigin = 0;
//    public static double wheel3YFromOrigin = 0;
//    public static double wheel3TixPerRotation = 28*20;
//    public static double wheel3Tuner = 1;
//    // Right
//    public static double wheel4XMovement = 90/25.4;
//    public static double wheel4YMovement = 0;
//    public static double wheel4XFromOrigin = 0;
//    public static double wheel4YFromOrigin = 0;
//    public static double wheel4TixPerRotation = 28*20;
//    public static double wheel4Tuner = 1;
}
