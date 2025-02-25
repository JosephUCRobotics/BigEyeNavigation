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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Drive", group = "A")
@Disabled
public class GamepadDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor front = hardwareMap.get(DcMotor.class, "front");
        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor back = hardwareMap.get(DcMotor.class, "back");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        DcMotor leftShoot = hardwareMap.get(DcMotor.class, "left shoot");
        DcMotor rightShoot = hardwareMap.get(DcMotor.class, "right shoot");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        Servo plunger = hardwareMap.get(Servo.class, "plunger");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front.setDirection(DcMotor.Direction.FORWARD);
        left.setDirection(DcMotor.Direction.REVERSE);
        back.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);

        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        CameraVariables.hardwareMap = hardwareMap;

        double frontPower;
        double leftPower;
        double backPower;
        double rightPower;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                plunger.setPosition(0.05);
            } else {
                plunger.setPosition(0.27);
            }

            frontPower = 1500*(gamepad1.left_stick_y + gamepad1.right_trigger-gamepad1.left_trigger * .3);
            leftPower = 1500*(gamepad1.left_stick_x - gamepad1.right_trigger+gamepad1.left_trigger * .3);
            backPower = 1500*(gamepad1.left_stick_y - gamepad1.right_trigger+gamepad1.left_trigger* .3);
            rightPower = 1500*(gamepad1.left_stick_x + gamepad1.right_trigger-gamepad1.left_trigger * .3);

            if (((DcMotorEx) front).getVelocity() > 0 && ((DcMotorEx) front).getVelocity() - frontPower>.05) {
                frontPower += .05;
            } else if (((DcMotorEx) front).getVelocity() < 0 && ((DcMotorEx) front).getVelocity() - frontPower<-.05) {
                frontPower -= .05;
            }

            if (((DcMotorEx) left).getVelocity() > 0 && ((DcMotorEx) left).getVelocity()- leftPower >.05) {
                leftPower += .05;
            } else if (((DcMotorEx) left).getVelocity() < 0 && ((DcMotorEx) left).getVelocity() - leftPower <-.05) {
                leftPower -= .05;
            }

            if (((DcMotorEx) back).getVelocity() > 0 && ((DcMotorEx) back).getVelocity() - backPower > .05) {
                backPower += .05;
            } else if (((DcMotorEx) back).getVelocity() < 0 && ((DcMotorEx) back).getVelocity() - backPower < -.05) {
                backPower -= .05;
            }

            if (((DcMotorEx) right).getVelocity() > 0 && ((DcMotorEx) right).getVelocity() - rightPower > .05) {
                rightPower += .05;
            } else if (((DcMotorEx) right).getVelocity() < 0 && ((DcMotorEx) right).getVelocity() - rightPower <- .05) {
                rightPower -= .05;
            }

            ((DcMotorEx) front).setVelocity(frontPower);
            ((DcMotorEx) left).setVelocity(leftPower);
            ((DcMotorEx) back).setVelocity(backPower);
            ((DcMotorEx) right).setVelocity(rightPower);


            if(gamepad1.y) {
                leftShoot.setPower(1);
                rightShoot.setPower(-1);
            } else {
                leftShoot.setPower(0);
                rightShoot.setPower(0);
            }

            double armPower = gamepad1.right_stick_y*.3;
            if (armPower > 0 && arm.getCurrentPosition() > -200 || armPower < 0 && arm.getCurrentPosition() < -3500){
                armPower = 0;
            }
            arm.setPower(armPower);

            telemetry.addData("Arm Pos:", arm.getCurrentPosition());
            telemetry.addData("Arm Pow", arm.getPower());
            telemetry.update();
        }
    }
}

