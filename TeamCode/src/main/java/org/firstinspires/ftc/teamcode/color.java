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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Color", group="LANbros")
public class color extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    ColorSensor CS;
    DeviceInterfaceModule CDI;




    //Raw value is a whole number between 0 and 1023
    static double odsReadngRaw;

    // odsReadinRaw to the power of (-0.5)
    static double odsReadingLinear;


    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        CS = hardwareMap.colorSensor.get("CS");
        CDI = hardwareMap.deviceInterfaceModule.get("CDI");

        waitForStart();

        while (opModeIsActive()) {

           if (CS.red() > CS.blue()) { // if the color sensor detects red then turn the blue LED on the Core Device Interface module on
               CDI.setLED(0,true);
           }    else { CDI.setLED(0,false); } // if it doesn't detect any red then keep the LED off

            if (CS.blue() > CS.red()) { // if the color sensor detects blue then turn the red LED on the Core Device Interface module on
                CDI.setLED(1,true);
            }    else { CDI.setLED(1,false); }

            telemetry.addData("RED", CS.red());
            telemetry.addData("BLUE", CS.blue());

            telemetry.update();
        }
    }
}//end class