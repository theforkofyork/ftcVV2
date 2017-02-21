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
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
public class LBHW implements PID_Constants {
    /* Public OpMode members. */
    public Servo gate = null;
    public Servo bleft = null;
    public Servo bright = null;
    public DcMotor mr1 = null;
    public DcMotor mr2 = null;
    public DcMotor ml2 = null;
    public DcMotor ml1 = null;
    public DcMotor fly = null;
    public DcMotor sweep = null;
    public OpticalDistanceSensor ODS = null;
    public OpticalDistanceSensor ODS2 = null;
    public ColorSensor CS = null;
    public OurRangeSensor rangeSensor = null;
    public OurRangeSensor rangeSensor2 = null;
    public I2cDeviceSynch rangeSensor2Reader = null;
    public I2cDevice rangeSensor2Device = null;
    public ModernRoboticsI2cGyro gyro = null;
    public ModernRoboticsI2cGyro gyro2 = null;
    public I2cDeviceSynch rangeSensorReader = null;
    public I2cDevice rangeSensorDevice = null;

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
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public LBHW() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        // Save reference to Hardware map
        hwMap = ahwMap;
        //  imu = new MasqAdafruitIMU("IMU", hwMap);
        // Define and Initialize Motors
        mr1 = hwMap.dcMotor.get("mr1");
        mr2 = hwMap.dcMotor.get("mr2");
        ml1 = hwMap.dcMotor.get("ml1");
        ml2 = hwMap.dcMotor.get("ml2");
        fly = hwMap.dcMotor.get("fly1");
        sweep = hwMap.dcMotor.get("sweep");
        gate = hwMap.servo.get("g");
        bright = hwMap.servo.get("b1");
        bleft = hwMap.servo.get("b2");
        CS = hwMap.colorSensor.get("CS");
        ODS = hwMap.opticalDistanceSensor.get("ODS");
        ODS2 = hwMap.opticalDistanceSensor.get("ODS2");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        gyro2 = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro2");
        fly.setDirection(DcMotor.Direction.REVERSE);
        sweep.setDirection(DcMotor.Direction.REVERSE);
        ml2.setDirection(DcMotor.Direction.REVERSE);
        mr2.setDirection(DcMotor.Direction.REVERSE);
        sweep.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rangeSensor2Device = hwMap.i2cDevice.get("range2");
        rangeSensor2Reader = new I2cDeviceSynchImpl(rangeSensor2Device, I2cAddr.create8bit(0x40), false);
        rangeSensorDevice = hwMap.i2cDevice.get("range");
        rangeSensorReader = new I2cDeviceSynchImpl(rangeSensorDevice, I2cAddr.create8bit(0x28), false);
        rangeSensor2 = new OurRangeSensor(rangeSensor2Reader, I2cAddr.create8bit(0x40));
        rangeSensor = new OurRangeSensor(rangeSensorReader, I2cAddr.create8bit(0x28));

        // Set all motors to zero power
        ml1.setPower(0);
        ml2.setPower(0);
        mr1.setPower(0);
        mr2.setPower(0);
        gyro2.setI2cAddress(I2cAddr.create8bit(0x22));

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.

    }

}


