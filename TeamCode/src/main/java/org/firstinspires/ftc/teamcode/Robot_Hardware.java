/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class Robot_Hardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // Define Servo objects
    private Servo leftGripperServo = null;
    private Servo rightGripperServo = null;
    // Define IMU object
    private IMU imu = null;

    private int eLeft, eRight, eLateral; // encoder values

    // If GoBilda is like REV, then
    // -1.0 is full left, which is -135 degrees
    // +1.0 is full right, which is +135 degrees

    // left Position Open
    static double leftServoPositionOpen = 0.5;
    // left Position Closed
    static double leftServoPositionClosed = 0.0;
    // right Position Open
    static double rightServoPositionOpen = 0.4;
    // right Position Closed
    static double rightServoPositionClosed = 0.8;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot_Hardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_rear");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_rear");

        leftGripperServo = myOpMode.hardwareMap.get(Servo.class,"leftGripperServo");
        rightGripperServo = myOpMode.hardwareMap.get(Servo.class, "rightGripperServo");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        resetEncoderValues();

        setGripperPositionOpen();
        // setLeftGripperPosition(0.0);
        // setRightGripperPosition(0.0);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   = -gamepad1.left_stick_y;  Note: pushing stick forward gives negative value
     * @param lateral =  gamepad1.left_stick_x;
     * @param yaw     =  gamepad1.right_stick_x;
     */
    public void driveRobot(double axial, double lateral, double yaw) {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw),1);

        leftFrontPower  /= max;
        rightFrontPower /= max;
        leftBackPower   /= max;
        rightBackPower  /= max;

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param lfPower
     * @param rfPower
     * @param lrPower
     * @param rrPower
     */
    public void setDrivePower(double lfPower, double rfPower, double lrPower, double rrPower) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lrPower);
        rightBackDrive.setPower(rrPower);
    }

    /**
     * Get the odometry dead wheel encoder values
     * Returns an array of 3 integer values
     * @return
     */
    public int[] getEncoderValues() {
        // Need to confirm these are how the odometry encoders are mapped

        int eValues[] = new int[3];
        eValues[0] = leftFrontDrive.getCurrentPosition();
        eValues[1] = rightFrontDrive.getCurrentPosition();
        eValues[2] = leftBackDrive.getCurrentPosition();

        return eValues;
    }

    public void resetEncoderValues() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLeftGripperPosition (double servoPosition) {
        leftGripperServo.setPosition(servoPosition);
    }

    public void setRightGripperPosition (double servoPosition) {
        rightGripperServo.setPosition(servoPosition);
    }

    public void setGripperPositionOpen() {
        leftGripperServo.setPosition(leftServoPositionOpen);
        rightGripperServo.setPosition(rightServoPositionOpen);
    }

    public void setGripperPositionClosed() {
        leftGripperServo.setPosition(leftServoPositionClosed);
        rightGripperServo.setPosition(rightServoPositionClosed);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
