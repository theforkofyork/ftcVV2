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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID_Constants;
import org.firstinspires.ftc.teamcode.AdafruitIMU;
import org.firstinspires.ftc.teamcode.Power;
import org.firstinspires.ftc.teamcode.Direction;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Clock;
import org.firstinspires.ftc.teamcode.DashBoard;

/**
 Works for both red and blue sides
 Only if close to corner vortex, if at the other spot then use AutonFar
 */

@Autonomous(name="Blue Auton Far (2B,2P)", group="Blue")
public class BlueAutonFar extends LinearOpMode implements PID_Constants {

    enum State {
        Drive_Forward,
        Shoot,
        Turn_To_Line,
        Drive,
        Drive_To_Line,
        Reverse_To_Line,
        Align,
        WallALign,
        WallAlign2,
        Align2,
        Detect_Color,
        Detect_Color2,
        Red_Beacon2,
        Blue_Beacon2,
        Red_Beacon,
        Blue_Beacon,
        Reverse,
        Reverse2,
        Turn_To_Beacon,
        Reverse_To_Line2,
        Drive_To_Line2,
        Park,
        Stop,
    }

    State state;

    /* Declare OpMode members. */
    LBHW robot = new LBHW ();
    AdafruitIMU imu = new AdafruitIMU("IMU");
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double SLOW_SPEED = 0.35;
    static final double STOP = 0;
    static double odsReadngRaw;
    static double odsReadingLinear;
    static double odsReadngRaw2;
    static double odsReadingLinear2;

    double TARGET_VOLTAGE = 12.5;
    double voltage = 0;
    double kP = 0.18;

    private double fTarget = 7.5e-7;
    private double fVelocity = 0.0;


    private int fEncoder = 0;
    private int fLastEncoder = 0;

    private long fVelocityTime = 0;
    private long fLastVelocityTime = 0;

    private double tolerance = 0.5e-7;
    private static final int DEFAULT_SLEEP_TIME = 1000;
    private static final double DEFAULT_TIMEOUT = 3;



    @Override
    public void runOpMode() throws InterruptedException {
        //MasqAdafruitIMU imu = new MasqAdafruitIMU("IMU", hardwareMap);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        state = State.Drive_Forward;


        int target = 0;  //Desired angle to turn to
        robot.ml1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      //  DashBoard dash = new DashBoard(telemetry);
        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        // make sure the gyro is calibrated.


        // wait for the start button to be pressed.


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            odsReadngRaw = robot.ODS.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);
            odsReadngRaw2 = robot.ODS2.getRawLightDetected();
            odsReadingLinear2 = Math.pow(odsReadngRaw2, -0.5);

            switch (state) {
                case Drive_Forward: {  //Drive forward for 0.7 seconds then stop and switch states to the turning coder
                    voltageshoot();
                    robot.gate.setPosition(1);
                    encoderDrive(SLOW_SPEED, 5.8, 5.8, 3);
                    state = State.Shoot;
                }
                break;
                case Shoot: {
                    shoot();
                    robot.fly.setPower(0);
                    state = State.Turn_To_Line;
                }
                break;

                case Turn_To_Line: {
                   imuRight(3,0.06);
                    state = State.Drive_To_Line;
                }
                break;
                case Drive: {

                    sleep(400);
                    state = State.Drive_To_Line;
                } break;
                case Drive_To_Line:
                    if (odsReadingLinear <= 1.5 ) {
                       // encoderDrive(SLOW_SPEED,0.15,0.15,3);
                       powerDrive(0.1);
                        sleep(200);
                        powerDrive(0);
                        state = State.Align;
                    }
                else {
                    powerDrive(0.25);
                }
                break;

                case Align: { //Align to the beacon by turning -85 degrees
                   imuRight(63,0.050);;
                    state = State.WallALign;

                }

                break;
                case WallALign: {
                    if (robot.rangeSensor.getDistance(DistanceUnit.CM) <= 12) {
                        encoderDrive(STOP,0,0,0);
                        state = State.Detect_Color;
                    }
                    else if (robot.rangeSensor.getDistance(DistanceUnit.CM) > 12) {
                        powerDrive(0.1);
                    }
                } break;
                case WallAlign2: {
                    if (robot.rangeSensor.getDistance(DistanceUnit.CM) <= 13) {
                        encoderDrive(STOP,0,0,0);
                        state = State.Detect_Color2;
                    }
                    else if (robot.rangeSensor.getDistance(DistanceUnit.CM) > 13) {
                        robot.leftDrive(0.04);
                        robot.rightDrive(0.1);
                    }
                } break;
                case Detect_Color: {
                    if (robot.CS.blue() > robot.CS.red()) { //If blue is detected then go to the blue detected state
                        state = State.Blue_Beacon;
                    } else if (robot.CS.blue() < robot.CS.red()) { //If red is detected then go to the red detected state
                        state = State.Red_Beacon;
                    } else {
                        encoderDrive(STOP,0,0,0);//if nothing is detected then stop the motors
                    }
                }
                break;
                case Red_Beacon: {
                    {
                        robot.bleft.setPosition(1);
                        robot.bright.setPosition(1);
                        powerDrive(-0.3);
                        sleep(400);
                        robot.bright.setPosition(0);
                        powerDrive(-0.2);
                        sleep(900);
                        powerDrive(0);
                        state = State.Turn_To_Beacon;
                    }
                }
                break;

                case Blue_Beacon: {
                    {
                        robot.bleft.setPosition(0);
                        robot.bright.setPosition(0);
                        powerDrive(-0.3);
                        sleep(400);
                        robot.bleft.setPosition(1);
                        powerDrive(-0.2);
                        sleep(900);
                        powerDrive(0);
                        state = State.Turn_To_Beacon;
                    }
                }
                break;

                case Reverse: {
                    encoderDrive(SLOW_SPEED, -5, -5, 3);
                    state = State.Turn_To_Beacon;
                }
                break;
                case Turn_To_Beacon: { //Align to the beacon by turning -85 degrees
                      imuRight(84,0.06);
                    state = State.Reverse2;

                }
                break;
                case Reverse2: {
                    powerDrive(0.45);
                    sleep(250);
                    state = State.Drive_To_Line2;
                } break;
                case Drive_To_Line2: // Drive to the white line
                    if (odsReadingLinear <= 1.5 || odsReadingLinear2 <= 1.5) { // Once the line is detected, stop the roobot and switch states
                        powerDrive(0.1);
                        sleep(800);
                        powerDrive(0);
                        state = State.Align2;
                    }
                {
                    powerDrive(0.25);
                }
                break;
                case Align2: {
                   imuLeft(113,0.06);
                    state = State.WallAlign2;

                }
                break;
                case Detect_Color2: {
                    if (robot.CS.blue() > robot.CS.red()) { //If blue is detected then go to the blue detected state
                        state = State.Blue_Beacon2;
                    } else if (robot.CS.blue() < robot.CS.red()) { //If red is detected then go to the red detected state
                        state = State.Red_Beacon2;
                    } else { //if nothing is detected then stop the motors
                        encoderDrive(STOP,0,0,0);
                    }

                }
                break;
                case Red_Beacon2: {
                    robot.bright.setPosition(1);
                    sleep(1000);
                    robot.bright.setPosition(0);
                    sleep(700);
                    robot.bright.setPosition(0.5);
                    robot.bleft.setPosition(0.5);
                    powerDrive(-0.2);
                    sleep(900);
                    powerDrive(0);
                    state = State.Park;
                }
                break;

                case Blue_Beacon2: {
                    robot.bleft.setPosition(0);
                    sleep(1000);
                    robot.bleft.setPosition(1);
                    sleep(700);
                    robot.bleft.setPosition(0.5);
                    robot.bright.setPosition(0.5);
                    powerDrive(-0.2);
                    sleep(900);
                    powerDrive(0);
                    state = State.Park;
                }
                break;

                case Park: {
                    encoderDrive(DRIVE_SPEED, -45, -45, 6);
                    state = State.Stop;
                }
                break;
                case Stop: {
                    encoderDrive(STOP, 0, 0, 0);
                }
                break;
            }

           // telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.addData("Roll",imu.getRoll());
            telemetry.addData("Pitch",imu.getPitch());
            telemetry.addData("TargetAngle", imu.getHeading());
            telemetry.update();


        }

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

      public void imuRight(int degs, double speed){
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double yawStart = yaw;
        // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
        while(Math.abs(yaw - ((yawStart - degs*.8) % 360)) > 2  && opModeIsActive()) {
            telemetry.addData("Target",((yawStart - degs) % 360));
            telemetry.addData("Progress",Math.abs(yaw - ((yawStart - degs) % 360)));
            telemetry.update();
            angles = imu.getAngles();
            yaw = angles[0];
            robot.ml1.setPower(-speed);
            robot.mr1.setPower(speed);
            robot.ml2.setPower(-speed);
            robot.mr2.setPower(speed);
        }
    }
  public void imuLeft(int degs, double speed){
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double yawStart = yaw;
        // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
        while(Math.abs(yaw - ((yawStart + degs*.8) % 360)) > 3  && opModeIsActive()) {
            telemetry.addData("Target",((yawStart - degs) % 360));
            telemetry.addData("Progress",Math.abs(yaw - ((yawStart - degs) % 360)));
            telemetry.update();
            angles = imu.getAngles();
            yaw = angles[0];
            robot.ml1.setPower(speed);
            robot.mr1.setPower(-speed);
            robot.ml2.setPower(speed);
            robot.mr2.setPower(-speed);
        }
    }

    public void turnAbsolute(int target) throws InterruptedException {
        MasqAdafruitIMU imu = new MasqAdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double pitch = angles[1];
        double roll = angles[2];
        double turnSpeed = 0.3;
        while (Math.abs(yaw - target*.8) > 1 && opModeIsActive()) {
            yaw = imu.getAngles()[0];
            //Continue while the robot direction is further than three degrees from the target
            if (yaw > target*.8) {  //if gyro is positive, we will turn right
                robot.ml1.setPower(-turnSpeed);
                robot.ml2.setPower(-turnSpeed);
                robot.mr1.setPower(turnSpeed);
                robot.mr2.setPower(turnSpeed);
            }
            if (yaw < target*.8) {  //if gyro is positive, we will turn left
                robot.ml1.setPower(turnSpeed);
                robot.ml2.setPower(turnSpeed);
                robot.mr1.setPower(-turnSpeed);
                robot.mr2.setPower(-turnSpeed);
            }

            if (Math.abs(yaw - target*.8) > 25) {

                turnSpeed = 0.1;
            } else {

                turnSpeed = 0.07;
            }


            telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.update();

        }

        {
            robot.ml1.setPower(0);
            robot.ml2.setPower(0);
            robot.mr1.setPower(0);
            robot.mr2.setPower(0);

        }
        //  sleep(250);   // optional pause after each move
    }




    public void noEncoder() throws InterruptedException {
        robot.mr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ml1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void powerDrive(double power) throws InterruptedException {
        robot.mr1.setPower(power);
        robot.mr2.setPower(power);
        robot.ml1.setPower(power);
        robot.ml2.setPower(power);
    }



    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newLeftTarget2;
        int newRightTarget2;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget2 = robot.ml2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTarget = robot.ml1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget2 = robot.mr2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.mr1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.ml1.setTargetPosition(newLeftTarget);
            robot.ml2.setTargetPosition(newLeftTarget);
            robot.mr2.setTargetPosition(newRightTarget);
            robot.mr1.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.ml1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ml2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.ml1.setPower(Math.abs(speed));
            robot.ml2.setPower(Math.abs(speed));
            robot.mr2.setPower(Math.abs(speed));
            robot.mr1.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.ml1.isBusy() && robot.mr1.isBusy() && (robot.ml2.isBusy() && robot.mr2.isBusy()))) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.ml1.getCurrentPosition(),
                        robot.ml2.getCurrentPosition(),
                        robot.mr2.getCurrentPosition(),
                        robot.mr1.getCurrentPosition());
                telemetry.addData("ODS linear", odsReadingLinear);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.ml1.setPower(0);
            robot.ml2.setPower(0);
            robot.mr2.setPower(0);
            robot.mr1.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setFPower(double power)
    {
        robot.fly.setPower(power);
    }

    public void bangBang()
    {
        fVelocityTime = System.nanoTime();
        fEncoder = robot.fly.getCurrentPosition();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime); //Use the encoder on the motor to force the flywheel to run at a constant speed

        if(fVelocity >= (fTarget + tolerance)) //if the motor is equal to or greater than the target speed then keep it at .95
        {
            setFPower(.95);
        }

        else if(fVelocity < (fTarget - tolerance)) //if the motor goes below the target speed, force it to run at full power
        {
            setFPower(1);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    } public void shoot(){
        robot.sweep.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.gate.setPosition(0.375);
        robot.sweep.setPower(0.35);
        sleep(1500);
        robot.gate.setPosition(1);
        robot.sweep.setPower(0);
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
        double error = TARGET_VOLTAGE - voltage; //error will be target voltage subtracted by current voltage
        double motorOut = (error * kP) + .9; //motor out is the error multiplied by our KP constant which is .18 and then it adds what ever error * kP to the target speed
        motorOut = Range.clip(motorOut, 0, 1); //make sure the motor doesn't go at a speed above 1
        setMotorPower(robot.fly, motorOut); // set the adjusted power to the motor

    }

}




