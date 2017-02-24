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

@Autonomous(name="Red Park and Shoot (0B,2P)", group="Red")
public class RedShootAuton extends LinearOpMode implements PID_Constants {

    enum State {
        Drive_Forward,
        Shoot,
        Park,
        Stop,
    }

    State state;

    /* Declare OpMode members. */
    LBHW robot = new LBHW ();
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

        robot.bright.setPosition(0.75);
        robot.bleft.setPosition(0.1);
        robot.gate.setPosition(0.95);


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


            switch (state) {
                case Drive_Forward: {  //Drive forward for 0.7 seconds then stop and switch states to the turning coder
                    sleep(10000);
                    voltageshoot();
                    robot.gate.setPosition(1);
                    encoderDrive(SLOW_SPEED, 10, 10, 3);
                    state = State.Shoot;
                }
                break;
                case Shoot: {
                    shoot();
                    robot.bright.setPosition(0.75);
                    robot.bleft.setPosition(0.1);
                    robot.fly.setPower(0);
                    state = State.Park;
                }
                break;
                case Park: {
                    encoderDrive(SLOW_SPEED, 15, 15, 6);
                    robot.bright.setPosition(0.75);
                    robot.bleft.setPosition(0.1);
                    state = State.Stop;
                }
                break;
                case Stop: {
                    encoderDrive(STOP, 0, 0, 0);
                    robot.bright.setPosition(0.75);
                    robot.bleft.setPosition(0.1);
                }
                break;
            }

        }

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
        robot.sweep.setPower(1);
        sleep(4000);
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
        double motorOut = (error * kP) + .95; //motor out is the error multiplied by our KP constant which is .18 and then it adds what ever error * kP to the target speed
        motorOut = Range.clip(motorOut, 0, 1); //make sure the motor doesn't go at a speed above 1
        setMotorPower(robot.fly, motorOut); // set the adjusted power to the motor

    }

}




