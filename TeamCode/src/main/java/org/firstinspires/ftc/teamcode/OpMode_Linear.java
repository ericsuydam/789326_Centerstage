/* Copyright (c) 2021 FIRST. All rights reserved.
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

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode Linear", group="Linear Opmode")

public class OpMode_Linear extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    Robot_Hardware robot       = new Robot_Hardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    enum Gripper_Position {
        OPEN,
        CLOSED
    }

    private Gripper_Position gripperPosition = Gripper_Position.OPEN;
    private double leftGripper_Position = 0.0;
    private double  rightGripper_Position = 0.0;

    private double[] robot_Pose = new double[3];

    @Override
    public void runOpMode() {

        robot.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gripperPosition = Gripper_Position.OPEN;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Note that robotHeading is returned in RADIANS
            double robotHeading = robot.getIMUHeading();
            // Convert to degrees
            double robotHeadingDegrees = robotHeading * 180.0 / Math.PI;
            /** After initialization, turning to the left results in a positive heading value
             *  and turning to the right results in a negative heading value;
             *  At +180, the heading converts to -180 and begins to count down,
             *  The heading will not accumulate.  It will never go beyond +180 or -180
            **/

            robot.updatePose();
            robot_Pose = robot.getPose();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_input   = -gamepad1.left_stick_y / 2.0;  // Note: pushing stick forward gives negative value
            double lateral_input =  gamepad1.left_stick_x / 2.0;
            double yaw_input     =  gamepad1.right_stick_x / 3.0;

            boolean actuate_gripper = gamepad1.right_bumper;
            boolean open_gripper = gamepad1.left_bumper;
            boolean left_increase = gamepad1.dpad_up;
            boolean left_decrease = gamepad1.dpad_down;
            boolean right_increase = gamepad1.dpad_right;
            boolean right_decrease = gamepad1.dpad_left;

            /**
            double axial_sign = axial_input / abs(axial_input);
            double lateral_sign = lateral_input / abs(lateral_input);
            double yaw_sign = yaw_input / abs(yaw_input);

            double axial_output = axial_sign * pow(axial_input, 2);
            double lateral_output = lateral_sign * pow(lateral_input, 2);
            double yaw_output = yaw_sign * pow(yaw_input,2);
            **/

            robot.driveRobot(axial_input, lateral_input, yaw_input);

            if (actuate_gripper) {
                if (gripperPosition == Gripper_Position.OPEN) {
                    robot.setGripperPositionClosed();
                    gripperPosition = Gripper_Position.CLOSED;
                }
            }

            if (open_gripper) {
                if (gripperPosition == Gripper_Position.CLOSED) {
                    robot.setGripperPositionOpen();
                    gripperPosition = Gripper_Position.OPEN;
                }
            }

            if (left_increase) {
                leftGripper_Position += 0.01;
                robot.setLeftGripperPosition(leftGripper_Position);
            }

            if (left_decrease) {
                leftGripper_Position -= 0.01;
                robot.setLeftGripperPosition(leftGripper_Position);
            }

            if (right_increase) {
                rightGripper_Position += 0.01;
                robot.setRightGripperPosition(rightGripper_Position);
            }

            if (right_decrease) {
                rightGripper_Position -= 0.01;
                robot.setRightGripperPosition(rightGripper_Position);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pose","X Location: " + robot_Pose[0]);
            telemetry.addData("Pose","Y Location: " + robot_Pose[1]);
            telemetry.addData("Pose","Heading: " + robot_Pose[2]);
            //telemetry.addData("IMU", "Heading" + robotHeadingDegrees);
            telemetry.addData("IMU","%.0f degrees", robotHeadingDegrees);
            telemetry.update();
        }
    }
}

