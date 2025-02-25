
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BigEyeDrive {
        double[] newArray = {0, 0, 0, 0};
        double[] wheels_Direction = newArray;
        double[] wheels_G = newArray;
        double[] wheels_Td = newArray;
        boolean moveIsActive = false;
        double[] wheels_currentTargetPos = newArray;
        double[] wheelsXMovement = BigEyeDriveVariables.wheelXMovement;
        double[] wheels_moveStartPos = newArray;
        double[] previousWheelsPos;
        public BigEyeDrive(){
                calculateWheelContents();
        }
        public void setMove(double x, double y, double a, double wheel1_encoderStartPos, double wheel2_encoderStartPos,
                            double wheel3_encoderStartPos,double wheel4_encoderStartPos){
                wheels_moveStartPos[0] = wheel1_encoderStartPos;
                wheels_moveStartPos[1] = wheel2_encoderStartPos;
                wheels_moveStartPos[2] = wheel3_encoderStartPos;
                wheels_moveStartPos[3] = wheel4_encoderStartPos;
                ElapsedTime moveTime = new ElapsedTime();

                double[] wheels_target = calculateTheWheelsMovement(x,y,a);


                // get the greatest target
                double greatestTarget = 0;
                for (double target : wheels_target) {
                        if (target > greatestTarget) {
                                greatestTarget = target;
                        }
                }

                double[] wheels_ratio = newArray;
                for(int i = 0; i < wheelsXMovement.length; i++){
                        wheels_ratio[i] = wheels_target[i] / greatestTarget;
                }


                moveIsActive = true;
                moveTime.reset();
                while(moveIsActive){
                        double []posAndVel = findCurrentTargetPos(1000,1000,2000,greatestTarget,moveTime.seconds());
                        for(int i = 0; i < wheelsXMovement.length; i++){
                                wheels_currentTargetPos[i] = posAndVel[0] * wheels_ratio[i];
                        }
                }
        }
        public void calculateWheelContents(){
                // calculate the wheel directions
                for(int i = 0; i < wheelsXMovement.length; i++){
                        if (wheelsXMovement[i] != 0){
                                wheels_Direction[i] = Math.toDegrees(Math.atan(BigEyeDriveVariables.wheelYMovement[i]/wheelsXMovement[i]));
                                if (wheelsXMovement[i] < 0){
                                        wheels_Direction[i] += 180;
                                }
                                if (wheels_Direction[i] > 180){
                                        wheels_Direction[i] -= 360;
                                } else if (wheels_Direction[i] < -180) {
                                        wheels_Direction[i] += 360;
                                }
                        } else {
                                if (BigEyeDriveVariables.wheelYMovement[i] > 0){
                                        wheels_Direction[i] = 90;
                                } else {
                                        wheels_Direction[i] = -90;
                                }
                        }
                }

                for(int i = 0; i < wheelsXMovement.length; i++){
                        // calculate the wheels G
                        wheels_G[i] = 90 - wheels_Direction[i];

                        // calculate the wheels total distends (Td)
                        // Tp = sqrt( xd^2 + yd^2 )
                        wheels_Td[i] = Math.sqrt(BigEyeDriveVariables.wheelXFromOrigin[i]*BigEyeDriveVariables.wheelXFromOrigin[i] + BigEyeDriveVariables.wheelYFromOrigin[i]*BigEyeDriveVariables.wheelYFromOrigin[i]);
                }
        }
        public double[] calculateTheWheelsMovement(double XTarget, double YTarget, double ATarget){
                double[] wheelsMovement = newArray;
                for(int i = 0; i < wheelsXMovement.length; i++) {
                        // calculate the wheels Y
                        double wheel_Y = Math.cos(Math.toRadians(wheels_G[i])) * YTarget;

                        // calculate the wheels X
                        double wheel_X = Math.cos(Math.toRadians(wheels_Direction[i])) * XTarget;

                        // calculate the wheels angle
                        // a = sqrt( 2Td^2 - 2Td^2 * cos(R) )
                        double wheel_a = Math.sqrt(2 * wheels_Td[i] * wheels_Td[i] - 2 * wheels_Td[i] * wheels_Td[i] * Math.cos(Math.toRadians(ATarget)));
                        // B = atan(yd/xd)
                        double wheel_B = Math.toDegrees(Math.atan(BigEyeDriveVariables.wheelYFromOrigin[i] / BigEyeDriveVariables.wheelXFromOrigin[i]));
                        // C = asin( Td*sin(R) )/a
                        double wheel_C = Math.toDegrees(Math.asin(wheels_Td[i] * Math.sin(Math.toRadians(ATarget)))) / wheel_a;
                        // D = C + B - Wd
                        double wheel_D = wheel_C + wheel_B - wheels_Direction[i];
                        // E = R / 2
                        double wheel_E = ATarget / 2;
                        // F = D + E
                        double wheel_F = wheel_D + wheel_E;
                        // Z = cos(F) * a
                        double wheel_Z = Math.cos(Math.toRadians(wheel_F)) * wheel_a;
                        wheelsMovement[i] = wheel_Y + wheel_X + wheel_Z;
                }

                return wheelsMovement;
        }
        public double[] findCurrentTargetPos(double acc, double dec, double maxV, double distance, double time) {
                double velocity;
                double currentTargetPos;
                boolean distanceWasNegative = false;
                // checks to see if distance is a negative number
                if (distance < 0) {
                        distance = Math.abs(distance);
                        distanceWasNegative = true;
                }

                double accT = maxV / acc;
                double decT = maxV / dec;
                double accD = maxV / 2 * accT;
                double decD = maxV / 2 * decT;
                double centerD = distance - (accD + decD);
                double centerT = centerD / maxV;
                // checks to see if the  move hase ended
                if (time > accT + centerT + decT) {
                        moveIsActive = false;
                        velocity = 0;
                        currentTargetPos = distance;
                // checks to see if the move can get up to the max velocity
                } else if (distance > accD + decD) {

                        if (time < accT) {
                                velocity = acc * time;
                                currentTargetPos = velocity / 2 * time;
                        } else if (time > accT + centerT) {
                                double segmentT = time - (accT + centerT);
                                velocity = maxV - dec * segmentT;
                                currentTargetPos = accD + centerD + (maxV + velocity) / 2 * segmentT;
                        } else {
                                velocity = maxV;
                                double segmentT = time - accT;
                                currentTargetPos = accD + maxV * segmentT;
                        }
                } else {
                        accD = dec / (acc + dec) * distance;
                        accT = Math.sqrt((2 * accD) / acc);
                        if (time < accT) {
                                velocity = acc * time;
                                currentTargetPos = velocity / 2 * time;
                        } else {
                                double segmentT = time - accT;
                                double peakV = acc * accT;
                                velocity = peakV - dec * segmentT;
                                currentTargetPos = accD + (peakV + velocity) / 2 * segmentT;
                        }
                }
                if (distanceWasNegative) {
                        velocity *= -1;
                        currentTargetPos *= -1;
                }
                return new double[]{currentTargetPos, velocity};
        }
        public double calculateWheelPower(int i, double pos){
                double inchPerRotation = Math.sqrt(Math.pow(wheelsXMovement[i], 2)+Math.pow(BigEyeDriveVariables.wheelYMovement[i], 2));
                double tixPerInch = BigEyeDriveVariables.wheelTixPerRotation[i] / inchPerRotation;
                double targetInTix = wheels_currentTargetPos[i] * tixPerInch;
                double err = targetInTix - (pos - wheels_moveStartPos[i]);
                //double wheelPower = err * 10 * BigEyeDriveVariables.wheelTuner[i];
                return err * 10 * BigEyeDriveVariables.wheelTuner[i];
        }
        public void updatePos(double[] wheelPos){
                if (previousWheelsPos != null){
                        for (int i = 0; i < wheelPos.length; i++) {

                        }
                }
                previousWheelsPos = wheelPos;
        }
}

