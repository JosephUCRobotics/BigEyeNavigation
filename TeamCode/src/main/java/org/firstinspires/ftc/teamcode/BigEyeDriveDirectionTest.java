package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BigEyeDriveDirectionTest", group="BigEyeDrive")
@Disabled
public class BigEyeDriveDirectionTest extends LinearOpMode {

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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double frontPower = .1;
            double leftPower = .1;
            double backPower = .1;
            double rightPower = .1;

            // Send calculated power to wheels
            frontDrive.setPower(frontPower);
            rightDrive.setPower(rightPower);
            leftDrive.setPower(leftPower);
            backDrive.setPower(backPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front/Right", "%4.2f, %4.2f", frontPower, rightPower);
            telemetry.addData("Left/Back", "%4.2f, %4.2f", leftPower, backPower);
            telemetry.update();
        }
    }}
