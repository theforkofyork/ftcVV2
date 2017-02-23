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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue Auton Far (1B,2P)", group="Blue")
public class BlueAutonFar extends LinearOpMode implements PID_Constants {

    enum State {
        Drive_Forward,
        Shoot,
        Turn_To_Line,
        Turn_To_LineRed,
        Drive,
        Drive_To_Line,
        Drive_To_LineRed,
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
        Balance,
        Balance2,
        ResetEncoder,
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
    static final double SLOW_SPEED = 0.4;
    static final double STOP = 0;
    static double odsReadngRaw;
    static double odsReadingLinear;
    static double odsReadngRaw2;
    static double odsReadingLinear2;



    double TARGET_VOLTAGE = 12.5;
    double voltage = 0;
    double kP = 0.18;






    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        state = State.Drive_Forward;
        resetEncoder();
        robot.bright.setPosition(0.75);
        robot.bleft.setPosition(0.1);
        robot.gate.setPosition(1);



        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        robot.gyro.calibrate();
        robot.gyro2.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && robot.gyro.isCalibrating() && robot.gyro2.isCalibrating()) {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.gyro.resetZAxisIntegrator();
        robot.gyro2.resetZAxisIntegrator();

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
                    resetEncoder();
                    sleep(100);
                    voltageshoot();
                    driveStraight(1200,0.4);
                    right(0.1);
                    sleep(900);
                    right(0);
                    state = State.Shoot;
                }
                break;
                case Turn_To_LineRed: {
                    state = State.Drive_To_LineRed;
                }
                break;
                
                case Shoot: {
                    shoot();
                    robot.fly.setPower(0);
                    state = State.Turn_To_Line;
                }
                break;

                case Turn_To_Line: {
                    rotateDegrees(18);
                    state = State.Drive;
                }
                break;
                case Drive: {
                    powerDrive(0.9);
                    sleep(800);
                    powerDrive(0.8);
                    sleep(400);
                    powerDrive(0.7);
                    sleep(200);
                    powerDrive(0.6);
                    sleep(200);
                    state = State.Drive_To_Line;
                } break;
                case Drive_To_Line:
                    if (odsReadingLinear <= 1.5 ) {
                       encoderDrive(0.25,1.45,1.45,5);
                        state = State.Align;
                    }
                else {
                    powerDrive(0.25);
                }
                break;

                case Align: { //Align to the beacon by turning -85 degrees
                    rotateDegrees(60);
                    state = State.Balance;
                }
                break;
                case Balance: {
                    //balance();
                    state = State.WallALign;
                } break;
                case Balance2: {
                   // balance();
                    state = State.WallAlign2;
                } break;
                case WallALign: {
                   // double cm2 = robot.rangeSensor2.getDistance(DistanceUnit.CM);
                    double cm = robot.rangeSensor2.getDistance(DistanceUnit.CM);
                    if (cm == 255) {
                        powerDrive(0);
                        continue;
                    }
                   if (cm > 8) {
                       powerDrive(0.12);
                       telemetry();
                    } else if (cm <= 8) {
                       right(0);
                       left(0);
                       resetEncoder();
                       sleep(50);
                        state = State.Detect_Color;
                    }
                } break;
                case WallAlign2: {
                    //double cm2 = robot.rangeSensor2.getDistance(DistanceUnit.CM);
                    double cm = robot.rangeSensor2.getDistance(DistanceUnit.CM);
                    if (cm == 255) {
                        powerDrive(0);
                        continue;
                    }
                    if (cm > 9 ) {
                        powerDrive(0.12);
                        telemetry();
                    } else if (cm <= 9) {
                        right(0);
                        left(0);
                        state = State.Detect_Color2;
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
                            robot.bright.setPosition(0);
                            sleep(2000);
                            robot.bright.setPosition(0.75);
                            state = State.Reverse;
                    }
                }
                break;
                case Blue_Beacon: {
                    {
                            robot.bleft.setPosition(1);
                            sleep(2000);
                            robot.bleft.setPosition(0.1);
                            state = State.Reverse;
                    }
                }
                break;
                case Reverse: {
                   encoderDrive(0.7,-2.8,-2.8,5);
                    state = State.Turn_To_Beacon;
                }
                break;
                case Turn_To_Beacon: { //Align to the beacon by turning -85 degrees
                    rotateDegrees(77);
                    state = State.Reverse2;
                }
                break;
                case Reverse2: {
                    powerDrive(0.8);
                    sleep(650);
                    state = State.Drive_To_Line2;
                } break;
                case Drive_To_Line2: // Drive to the white line
                    if (odsReadingLinear2 <= 1.5) { // Once the line is detected, stop the roobot and switch states
                        encoderDrive(0.25,0.9,0.9,5);
                        state = State.Align2;
                    }
                {
                    powerDrive(0.25);
                }
                break;
                case Align2: {
                    rotateDegrees(-85);
                    state = State.Balance2;

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
                    right(0.12);
                    sleep(650);
                    right(0);
                    robot.bright.setPosition(0);
                    sleep(2000);
                    robot.bright.setPosition(0.75);
                    state = State.Stop;
                }
                break;

                case Blue_Beacon2: {
                    left(0.12);
                    sleep(550);
                    left(0);
                    robot.bleft.setPosition(1);
                    sleep(2000);
                    robot.bleft.setPosition(0.1);
                    state = State.Stop;
                }
                break;

                case Park: {
                    resetEncoder();
                    encoderDrive(SLOW_SPEED, -45, -45, 6);
                    state = State.Stop;
                }
                break;
                case Stop: {
                    encoderDrive(STOP, 0, 0, 0);
                }
                break;
            }




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


    public void noEncoder()  {
        robot.ml1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoder() {
        robot.ml1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void powerDrive(double power)  {
        runEncoder();
        robot.mr1.setPower(power);
        robot.mr2.setPower(power);
        robot.ml1.setPower(power);
        robot.ml2.setPower(power);
    }

    public void powerDriveNoEncoder(double power)  {
        noEncoder();
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
            robot.ml2.setTargetPosition(newLeftTarget2);
            robot.mr2.setTargetPosition(newRightTarget2);
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
    public void rotateDegrees(int desiredDegrees) throws InterruptedException {
        desiredDegrees %= 360;
        if (1 >= Math.abs(desiredDegrees)) {
            return;
        }
        robot.gyro.resetZAxisIntegrator();
        robot.gyro2.resetZAxisIntegrator();
        double power = 0.15;
        boolean quit = false;
        while(opModeIsActive() && !quit) {
            runEncoder();
            if (0 < desiredDegrees) { // turning right, so heading should get smaller
                right(0.15);
                left(-0.15);
            } else { // turning left, so heading gets bigger.
                right(-0.15);
                left(0.15);
            }
            int value = robot.gyro.getIntegratedZValue() + robot.gyro2.getIntegratedZValue();
            int zValue = value / 2;
            final int currentHeading = - zValue;
            final int headingDiff = Math.abs(desiredDegrees - currentHeading);
            telemetry.addData("Headings", String.format("Target: %d, Current: %d", desiredDegrees, currentHeading));
            quit = headingDiff <= 5;
        }
        right(0);
        left(0);
    }
    public void runEncoder()  {
        robot.ml1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ml2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

 public void shoot()  {
        robot.sweep.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.gate.setPosition(0.1);
        robot.sweep.setPower(1);
        sleep(2000);
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

    public void balance()  {
        runtime.reset();
        while (Math.abs(robot.rangeSensor.getDistance(DistanceUnit.CM) - robot.rangeSensor2.getDistance(DistanceUnit.CM)) > 1 && opModeIsActive() && runtime.seconds() < 3) {
            double cm2 = robot.rangeSensor2.getDistance(DistanceUnit.CM);
            double cm = robot.rangeSensor.getDistance(DistanceUnit.CM);
            if (cm == 255 || cm2 == 255) {
                right(0);
                left(0);
                continue;
            }
            if (cm > cm2) {
                left(-1);
                right(1);
            }
            if (cm < cm2) {
                right(-1);
                left(1);
            }
            right(0);
            left(0);
            telemetry();
            idle();
        }
    }
    public void driveStraight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        double target = robot.gyro.getIntegratedZValue() + robot.gyro2.getIntegratedZValue()/2;  //Starting direction
        double startPosition = robot.ml1.getCurrentPosition() + robot.mr1.getCurrentPosition()/2;  //Starting position
        runEncoder();
        while (robot.ml1.getCurrentPosition()+robot.mr1.getCurrentPosition()/2 < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            int value = robot.gyro.getIntegratedZValue() + robot.gyro2.getIntegratedZValue();
            int zValue = value / 2;

            leftSpeed = power + (zValue - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (zValue - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            left(leftSpeed);
            right(rightSpeed);

            telemetry.addData("3. Distance to go", duration + startPosition - robot.ml1.getCurrentPosition());
            telemetry.addData("Zval",zValue);
            telemetry.update();

        }

        powerDrive(0);
    }

    public final void right(double power) //maybe not neccessary
    {
        robot.mr1.setPower(power);
        robot.mr2.setPower(power);
    }
    public final void left(double power) //maybe not neccessary
    {
        robot.ml1.setPower(power);
        robot.ml2.setPower(power);
    }
    public void telemetry(){
        telemetry.addData("Range1", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Range2", "%.2f cm", robot.rangeSensor2.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}




