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
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    // Define Servo objects
    private Servo GripperServo = null;
    private Servo leftGripperAngleServo = null;
    private Servo rightGripperAngleServo = null;
    // Define IMU object
    private IMU imu = null;
    // Define Sensors
    private TouchSensor leftTouchSensor = null;

    // Define and initialize odometry variables
    private int current_encoderValues[] = new int[3];
    private int previous_encoderValues[] = new int[3];
    private int delta_encoderValues[] = new int[3];
    // delta_movement is delta_encoderValues * WHEEL_CIRCUMFERENCE in millimeters
    private double delta_movement[] = new double[3];
    // Pose is the x, y, and heading of the robot on the field, relative to the start location & orientation
    private double current_Pose[] = new double[3];
    private double previous_Pose[] = new double[3];
    private double delta_Pose[] = new double[3];
    // Define and initialize physical odometry measurements
    // @TRACKWIDTH is the distance between the left and right odometry wheels
    public static final double TRACKWIDTH = 320.0;
    /** @FOREWARDOFFSET is the distance from the center of rotation to the lateral odometry wheel
     * If the lateral wheel is forward of the center of rotation, the value is positive
     * If the lateral wheel is behind the center of rotation, the value is negative
    */
    public static final double FOREWARDOFFSET = -160.0;
    public static final double WHEEL_CIRCUMFERENCE = 50.0 * Math.PI;

    // Position Open
    static double gripperServoPositionOpen = 0.5;
    // Position Closed
    static double gripperServoPositionClosed = 0.0;
    // left Position Open
    static double leftGripperAngleServoPositionOpen = 0.4;
    // left Position Closed
    static double leftGripperAngleServoPositionClosed = 0.8;
    // right Position Open
    static double rightGripperAngleServoPositionOpen = 0.4;
    // right Position Closed
    static double rightGripperAngleServoServoPositionClosed = 0.8;

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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor = myOpMode.hardwareMap.get(DcMotor.class, "left_arm");
        rightArmMotor = myOpMode.hardwareMap.get(DcMotor.class, "right_arm");
        
        // Define and initialize Servos
        gripperServo = myOpMode.hardwareMap.get(Servo.class,"gripperServo");
        leftGripperAngleServo = myOpMode.hardwareMap.get(DcMotor.class, "leftGripperAngleServo")
        rightGripperAngleServo = myOpMode.hardwareMap.get(Servo.class, "rightGripperAngleServo");

        // Define and Initialize Sensors
        leftTouchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "leftTouchSensor");

        // Define and initialize IMU
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        // Call resetEncoderValues to reset all three at once
        resetEncoderValues();
        updatePose();

        // Set the initial state of the Gripper to be Open
        setGripperPositionOpen();

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
    public void driveRobot(double axial_input, double lateral_input, double yaw_input) {

        // Convert raw gamepad inputs to outputs
        int axial_sign = 1;
        int lateral_sign = 1;
        int yaw_sign = 1;

        // capture the sign of the inputs before modifying
        if (axial_input < 0) {
            axial_sign = -1;
        }
        if (lateral_input < 0) {
            lateral_sign = -1;
        }
        if (yaw_input < 0) {
            yaw_sign = -1;
        }

        // Modify the raw inputs
        double axial = Math.pow(axial_input, 2) * axial_sign;
        double lateral = Math.pow(lateral_input, 2) * lateral_sign;
        double yaw = Math.pow(yaw_input, 2) * yaw_sign;

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
        return current_encoderValues;
    }

    public double[] getPose() {
        return current_Pose[];
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

    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     *
     */
    public void updatePose() {

        // enum to help keep the array positions clear
        int LEFT = 0;
        int RIGHT = 1;
        int LATERAL = 2;
        int X = 0;
        int Y = 1;
        int PHI = 2;

        // Store current values in previous values
        previous_encoderValues = current_encoderValues;
        previous_Pose = current_Pose;

        // Get updated encoderValues
        current_encoderValues = getEncoderValues();

        // update deltaEncoderValues.  Unsure if subtracting one array from another would also work
        for (int i = 0; i < current_encoderValues.length; i++) {
            delta_encoderValues[i] = current_encoderValues[i] - previous_encoderValues[i];
        }

        // update delta_movement
        for (int i = 0; i < delta_movement.length; i++) {
            delta_movement[i] = delta_encoderValues[i] * WHEEL_CIRCUMFERENCE;
        }

        // update delta_Pose
        delta_Pose[PHI] = (delta_movement[LEFT] - delta_movement[RIGHT]) * TRACKWIDTH;
        delta_Pose[X] = (delta_movement[LEFT] + delta_movement[RIGHT]) / 2.0;
        delta_Pose[Y] = (delta_movement[LATERAL] - (FOREWARDOFFSET * delta_Pose[PHI]));

        // update current_Pose
        current_Pose[X] = previous_Pose[X] + delta_Pose[X];
        current_Pose[Y] = previous_Pose[Y] + delta_Pose[Y];
        current_Pose[PHI] = previous_Pose[PHI] + delta_Pose[PHI];

    }
}
