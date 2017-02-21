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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Sensor: MR range sensor", group = "Sensor")
public class MRI_Range_Sensors extends LinearOpMode {

    DcMotor mr1;
    DcMotor mr2;
    DcMotor ml2;
    DcMotor ml1;
    OurRangeSensor rangeSensor;
    OurRangeSensor rangeSensor2;
    I2cDeviceSynch rangeSensor2Reader;
    I2cDevice rangeSensor2Device;

    I2cDeviceSynch rangeSensorReader;
    I2cDevice rangeSensorDevice;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {


      
        mr1 = hardwareMap.dcMotor.get("mr1");
        mr2 = hardwareMap.dcMotor.get("mr2");
        ml1 = hardwareMap.dcMotor.get("ml1");
        ml2 = hardwareMap.dcMotor.get("ml2");
        ml1.setDirection(DcMotor.Direction.REVERSE);
        mr1.setDirection(DcMotor.Direction.REVERSE);
        ml1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ml2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rangeSensor2Device = hardwareMap.i2cDevice.get("range2");
        rangeSensor2Reader = new I2cDeviceSynchImpl(rangeSensor2Device, I2cAddr.create8bit(0x40), false);
        rangeSensorDevice = hardwareMap.i2cDevice.get("range");
        rangeSensorReader = new I2cDeviceSynchImpl(rangeSensorDevice, I2cAddr.create8bit(0x28), false);
        rangeSensor2 = new OurRangeSensor(rangeSensor2Reader,I2cAddr.create8bit(0x40));
        rangeSensor = new OurRangeSensor(rangeSensorReader, I2cAddr.create8bit(0x28));
        telemetry.addData("Initialization ", "complete");
        telemetry.update();

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            balance();


            telemetry();
        }
    }

    public void balance() {
        runtime.reset();
        while (Math.abs(rangeSensor.getDistance(DistanceUnit.CM) - rangeSensor2.getDistance(DistanceUnit.CM)) > 1 && opModeIsActive() && runtime.seconds() < 3) {
            double cm2 = rangeSensor2.getDistance(DistanceUnit.CM);
            double cm = rangeSensor.getDistance(DistanceUnit.CM);
            if (cm == 255 || cm2 == 255) {
                right(0);
                continue;
            }
            if (cm > cm2) {
                left(1);
                right(-1);
            }
            if (cm < cm2) {
                right(1);
                 left(-1);
            }
            right(0);
            left(0);
            telemetry();
            idle();
        }
    }
    public final void right(double power) //maybe not neccessary
    {
        mr1.setPower(power);
        mr2.setPower(power);
    }
    public final void left(double power) //maybe not neccessary
    {
        ml1.setPower(power);
        ml2.setPower(power);
    }
    public void telemetry(){
        telemetry.addData("Range1", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Range2", "%.2f cm", rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}


