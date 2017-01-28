package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TTest", group="LANbros")
public class test extends OpMode {


    DcMotor fly2;
    DcMotor fly1;

    public test() {

    }


    @Override
    public void init() {


        fly1 = hardwareMap.dcMotor.get("fly1");
        fly2 = hardwareMap.dcMotor.get("fly2");
        fly1.setDirection(DcMotor.Direction.REVERSE);


    }


    @Override
    public void loop() {




    /*if (gamepad1.a) {
        speed = 0.75;
    }*/

        //fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.


        if (gamepad1.a) {
            fly1.setPower(1);
            fly2.setPower(1);
        } else if (gamepad1.b) {
            fly1.setPower(-1);
            fly2.setPower(-1);
        } else {
            fly1.setPower(0);
            fly2.setPower(0);
        }


    }

    public void stop() {

    }


}