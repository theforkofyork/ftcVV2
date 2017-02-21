package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="servos", group="LANbros")
@Disabled
public class servos extends OpMode {

    double speed = 1;

    Servo gate;
    Servo bleft;
    Servo bright;


    public servos() {

    }


    @Override
    public void init() {


        gate = hardwareMap.servo.get("g");
        bright = hardwareMap.servo.get("b1");
        bleft = hardwareMap.servo.get("b2");
        gate.setPosition(1);
        bright.setPosition(0.75);
        bleft.setPosition(0.2);

    }


    @Override
    public void loop() {



        telemetry.addData("Gate Position", gate.getPosition());
        telemetry.addData("B Right Position", bright.getPosition());
        telemetry.addData("B left Position", bleft.getPosition());
    }

    public void stop() {

    }


}