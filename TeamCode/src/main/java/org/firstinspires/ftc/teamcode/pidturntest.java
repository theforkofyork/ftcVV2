/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@Autonomous(name = "pid turn test", group = "Sensor")
public class pidturntest extends LinearOpMode implements PID_Constants {



    private ElapsedTime runtime = new ElapsedTime();
    LBHW robot = new LBHW();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int target = 0;
        resetEncoder();
        idle();



        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        robot.gyro2.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && robot.gyro.isCalibrating() && robot.gyro2.isCalibrating()) {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.gyro.resetZAxisIntegrator();
        robot.gyro2.resetZAxisIntegrator();

        // wait for the start button to be pressed
        waitForStart();
        while (opModeIsActive()) {
            rotateDegrees(10);
            int value = robot.gyro.getIntegratedZValue() + robot.gyro2.getIntegratedZValue();
            int zValue = value / 2;
            telemetry.addData(" val. %03d", value);
            telemetry.addData("Int. Ang. %03d", zValue);
            telemetry.addData("gyro2",robot.gyro2.getIntegratedZValue());
            telemetry.addData("gyro",robot.gyro.getIntegratedZValue());
            telemetry.update();
        }
    }



    public final void right(double power) //maybe not neccessary
    {
        robot.mr1.setPower(power);
        robot.mr2.setPower(power);
    }
    public final void left(double power) //maybe not neccessary
    {
        robot.ml1.setPower(power);
        robot.ml2.setPower(power);
    }
    public void telemetry(){

    }
    public void rotateDegrees(int desiredDegrees) throws InterruptedException {
        // Sorry. You can't just spin around.

        desiredDegrees %= 360;

        if (1 >= Math.abs(desiredDegrees)) {
            return;
        }

        robot.gyro.resetZAxisIntegrator();
        robot.gyro2.resetZAxisIntegrator();

        double power = 0.15;

        boolean quit = false;
        while(opModeIsActive() && !quit) {
            if (0 < desiredDegrees) { // turning right, so heading should get smaller
                right(0.15);
                left(-0.15);
            } else { // turning left, so heading gets bigger.
                right(-0.15);
                left(0.15);
            }
            int value = robot.gyro.getIntegratedZValue() + robot.gyro2.getIntegratedZValue();
            int zValue = value / 2;
            final int currentHeading = - zValue;
            final int headingDiff = Math.abs(desiredDegrees - currentHeading);

            telemetry.addData("Headings", String.format("Target: %d, Current: %d", desiredDegrees, currentHeading));

            quit = headingDiff <= 5;

        }
        right(0);
        left(0);
    }
    public void resetEncoder() {
        robot.ml1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


