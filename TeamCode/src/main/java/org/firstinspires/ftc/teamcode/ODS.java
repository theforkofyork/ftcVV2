/*
Modern Robotics Color Sensor Active & Passive Example
Created 8/9/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.75
Reuse permitted with credit where credit is due

Configuration:
Optical Distance sensor named "ods"
Left drive train motor named "ml"  (two letters)
Right drive train motor named "mr"
Both motors need encoders

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="ODS", group="LANbros")
public class ODS extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;
    DeviceInterfaceModule CDI;




    //Raw value is a whole number between 0 and 1023
    static double odsReadngRaw;

    // odsReadinRaw to the power of (-0.5)
    static double odsReadingLinear;


    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");
        CDI = hardwareMap.deviceInterfaceModule.get("CDI");

        waitForStart();

        while (opModeIsActive()) {

            odsReadngRaw = ODS.getRawLightDetected();                //update raw value
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);                //calculate linear value


            if (odsReadingLinear <= 1.85) { // if the white line is detected, turn the red LED on
                CDI.setLED(0,true);
            } else { CDI.setLED(0,false); } // if the white line isn't detected then keep the LED off
            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.


            telemetry.addData("ODS linear", odsReadingLinear);
            telemetry.addData("ODS detected", ODS.getLightDetected());
            telemetry.update();
        }
    }
}//end class