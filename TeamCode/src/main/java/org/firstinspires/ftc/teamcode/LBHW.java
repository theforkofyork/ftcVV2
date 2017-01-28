package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 */
public class LBHW implements PID_Constants
{
    /* Public OpMode members. */
    public Servo gate = null;
    public Servo bleft = null;
    public Servo bright = null;
    public DcMotor mr1 = null;
    public DcMotor mr2 = null;
    public DcMotor ml2 = null;
    public   DcMotor ml1 = null;
    public  DcMotor fly = null;
    public   DcMotor sweep = null;
    public   DcMotor lift1 = null;
    public  DcMotor lift2 = null;
    public OpticalDistanceSensor ODS = null;
    public OpticalDistanceSensor ODS2 = null;
    public ColorSensor CS = null;
    public  ModernRoboticsI2cRangeSensor rangeSensor = null;

    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;


    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double tolerance = 0.5e-7;
    private static final int DEFAULT_SLEEP_TIME = 1000;
    private static final double DEFAULT_TIMEOUT = 3;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public LBHW(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        AdafruitIMU imu = new AdafruitIMU("IMU");
      //  imu = new MasqAdafruitIMU("IMU", hwMap);
        // Define and Initialize Motors
        mr1 = hwMap.dcMotor.get("mr1");
        mr2 = hwMap.dcMotor.get("mr2");
        ml1 = hwMap.dcMotor.get("ml1");
        ml2 = hwMap.dcMotor.get("ml2");
        fly = hwMap.dcMotor.get("fly1");
        sweep = hwMap.dcMotor.get("sweep");
        lift1 = hwMap.dcMotor.get("lift1");
        lift2 = hwMap.dcMotor.get("lift2");
        gate = hwMap.servo.get("g");
        bright = hwMap.servo.get("b1");
        bleft = hwMap.servo.get("b2");
        CS = hwMap.colorSensor.get("CS");
        ODS = hwMap.opticalDistanceSensor.get("ODS");
        ODS2 = hwMap.opticalDistanceSensor.get("ODS2");
        fly.setDirection(DcMotor.Direction.REVERSE);
        sweep.setDirection(DcMotor.Direction.REVERSE);
        ml2.setDirection(DcMotor.Direction.REVERSE);
        mr2.setDirection(DcMotor.Direction.REVERSE);
        sweep.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // Set all motors to zero power
        ml1.setPower(0);
        ml2.setPower(0);
        mr1.setPower(0);
        mr2.setPower(0);
        bleft.setPosition(0.5);
        bright.setPosition(0.5);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        ml1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ml2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Define and initialize ALL installed servos.

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
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
            setFPower(1);
        }

        else if(fVelocity < (fTarget - tolerance))
        {
            setFPower(1);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }
    public void leftDrive(double power){
        ml1.setPower(power);
        ml2.setPower(power);
    }
    public void rightDrive(double power){
        mr1.setPower(power);
        mr2.setPower(power);
    }
    public LBHW(Telemetry telemetry){
        this.telemetry  = telemetry;
        instance = this;
    }
    public static LBHW getTelemetry(){
        return instance;
    }
    private static LBHW instance;
    private Telemetry telemetry;
    public void addTelemetry(String string) {
        telemetry.addLine(string);
    }
    public void addTelemetry(String string, Object data) {
        telemetry.addData(string, data);
    }
    public void addSticky(String string){
        telemetry.log().add(string);
        telemetry.update();
    }
    public void addSticky(String string, Object data){
        telemetry.log().add(string, data);
        telemetry.update();
    }
    private boolean opModeIsActive() {
        return ((LinearOpMode) (FtcOpModeRegister.opModeManager.getActiveOpMode())).opModeIsActive();
    }
    public void turnPID(double power, int angle, Direction DIRECTION, double timeOut,  int sleepTime) {
        AdafruitIMU imu = new AdafruitIMU("IMU");
        double targetAngle = imu.adjustAngle(imu.getHeading() + (DIRECTION.value * angle));
        double acceptableError = 0.5;
        double currentError = 1;
        double prevError = 0;
        double integral = 0;
        double newPower = power;
        double previousTime = 0;
        Clock clock = new Clock("clock");
        while (opModeIsActive() && (imu.adjustAngle(Math.abs(currentError)) > acceptableError) && !clock.elapsedTime(timeOut, Clock.Resolution.SECONDS)) {
            double tChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            tChange = tChange / 1e9;
            double imuVAL = imu.getHeading();
            currentError = imu.adjustAngle(targetAngle - imuVAL);
            integral += currentError  * ID;
            double errorkp = currentError * KP_TURN;
            double integralki = currentError * KI_TURN * tChange;
            double dervitive = (currentError - prevError) / tChange;
            double dervitivekd = dervitive * KD_TURN;
            newPower = (errorkp + integralki + dervitivekd);
            mr1.setPower(-newPower);
            mr2.setPower(-newPower);
            ml1.setPower(newPower);
            ml2.setPower(newPower);
            prevError = currentError;
            LBHW.getTelemetry().addTelemetry("TargetAngle", targetAngle);
           LBHW.getTelemetry().addTelemetry("Heading", imuVAL);
            LBHW.getTelemetry().addTelemetry("AngleLeftToCover", currentError);
            telemetry.update();
        }
        rightDrive(0);
        leftDrive(0);
        sleep(sleepTime);
    }
    public void sleep() {
        sleep(1000);
    }

    public void sleep(int time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void turnPID(double power, int angle, Direction DIRECTION, double timeout)  {
        turnPID(power, angle, DIRECTION, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turnPID(double power, int angle, Direction DIRECTION)  {
        turnPID(power, angle, DIRECTION, DEFAULT_TIMEOUT);
    }
}


