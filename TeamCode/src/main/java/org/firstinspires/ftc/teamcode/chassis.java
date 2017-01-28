package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Tele-Op", group="LANbros")
public class chassis extends OpMode {

    double speed = 1;

    Servo gate;
    Servo bleft;
    Servo bright;
    DcMotor mr1;
    DcMotor mr2;
    DcMotor ml2;
    DcMotor ml1;
    DcMotor fly;
    DcMotor sweep;
    DcMotor lift1;
    DcMotor lift2;

    public chassis() {

    }
    double TARGET_VOLTAGE = 12.5;
    double voltage = 0;
    double kP = 0.18;
    boolean IS_GATE_OPEN = false;


    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;


    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double tolerance = 0.5e-7;


    @Override
    public void init() {


        mr1 = hardwareMap.dcMotor.get("mr1");
        mr2 = hardwareMap.dcMotor.get("mr2");
        ml1 = hardwareMap.dcMotor.get("ml1");
        ml2 = hardwareMap.dcMotor.get("ml2");
        fly = hardwareMap.dcMotor.get("fly1");
        sweep = hardwareMap.dcMotor.get("sweep");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        gate = hardwareMap.servo.get("g");
        bright = hardwareMap.servo.get("b1");
        bleft = hardwareMap.servo.get("b2");
        fly.setDirection(DcMotor.Direction.REVERSE);
        sweep.setDirection(DcMotor.Direction.REVERSE);
        ml1.setDirection(DcMotor.Direction.REVERSE);
        mr1.setDirection(DcMotor.Direction.REVERSE);
        sweep.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    @Override
    public void loop() {
        sweep.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // write the values to the motors
        mr1.setPower(right);
        mr2.setPower(right);
        ml1.setPower(left);
        ml2.setPower(left);


        if (gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
            voltageshoot();
        } else if (gamepad1.right_bumper || gamepad2.right_bumper)
            setFPower(-1);
        else
           setFPower(0);

        if (gamepad1.a || gamepad2.a) {
            IS_GATE_OPEN = false; //set is gate open to false
            gate.setPosition(1); //keep gate closed
        } else if (gamepad1.b || gamepad2.b) {
            IS_GATE_OPEN = true; //set is gate open to true
            gate.setPosition(0.375); //open the gate
        }
        if (gamepad1.left_bumper || gamepad2.left_bumper)
            sweep.setPower(-1);
        else if (gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25)
            sweep.setPower(1);
        else if (IS_GATE_OPEN) { //if IS_GATE_OPEN is set to true then run the sweeper at 0.35
            sweep.setPower(0.35);
        } else
            sweep.setPower(0);

        if (gamepad1.dpad_left) {
            bleft.setPosition(0);
        } else if (gamepad1.dpad_right) {
            bleft.setPosition(1);
        } else if (gamepad1.x) {
            bright.setPosition(1);
        } else if (gamepad1.y) {
            bright.setPosition(0);
        }
        else {
            bleft.setPosition(0.5);
            bright.setPosition(0.5);
        }

        if (gamepad1.dpad_up) {
            lift1.setPower(1);
            lift2.setPower(1);
        } else if (gamepad1.dpad_down) {
            lift1.setPower(-1);
            lift2.setPower(-1);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Voltage",batteryVoltage());
        telemetry.addData("Target Voltage",TARGET_VOLTAGE);
    }

    public void stop() {

    }


    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public void setFPower(double power)
    {
        fly.setPower(power);
    }

    public void bangBang()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = fly.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if(fVelocity >= (fTarget + tolerance))
        {
            setFPower(0.8);
        }

        else if(fVelocity < (fTarget - tolerance))
        {
            setFPower(0.8);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }
    public double batteryVoltage()
    {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
    public final void setMotorPower(DcMotor motor, double power) //maybe not neccessary
    {
        motor.setPower(power);
    }
    public void voltageshoot() {
        voltage = batteryVoltage();
        double error = TARGET_VOLTAGE - voltage;
        double motorOut = (error * kP) + .82;
        motorOut = Range.clip(motorOut, 0, 1);
        setMotorPower(fly, motorOut);
    }
}