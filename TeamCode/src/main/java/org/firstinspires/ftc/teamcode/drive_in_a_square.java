package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Track Pos", group="drive")
public class drive_in_a_square extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor backDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontDrive  = hardwareMap.get(DcMotor.class, "front");
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        backDrive = hardwareMap.get(DcMotor.class, "back");

        BigEyeDrive drive = new BigEyeDrive();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //.setMove(0,18, 0, frontDrive.getCurrentPosition(),leftDrive.getCurrentPosition(),backDrive.getCurrentPosition(),rightDrive.getCurrentPosition());
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            double frontPower = drive.calculateWheelPower(0, frontDrive.getCurrentPosition());
//            double leftPower = drive.calculateWheelPower(1, leftDrive.getCurrentPosition());
//            double backPower = drive.calculateWheelPower(2, backDrive.getCurrentPosition());
//            double rightPower = drive.calculateWheelPower(3, rightDrive.getCurrentPosition());
//
//            // Send calculated power to wheels
//            frontDrive.setPower(frontPower);
//            rightDrive.setPower(rightPower);
//            leftDrive.setPower(leftPower);
//            backDrive.setPower(backPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontPower, frontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", frontPower, frontPower);
            telemetry.update();
        }
    }}
