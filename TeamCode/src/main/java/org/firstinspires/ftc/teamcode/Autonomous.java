package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Preston on 9/19/18
 */


//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear Opmode")
//@Disabled
public class Autonomous {
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    static final double WHEEL_RADIUS = 6;
//    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
//    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
//    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415926);
//
//
//    Robot robot = new Robot();
//    Thread go, mech;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //Initialize motors and their individual direction
//        telemetry.addData("Status:", "Initializing");
//
//        robot.init(hardwareMap);
//
//        telemetry.addData("Status:", "Initialized");
//        telemetry.update();
//        robot.setEncoder();
//
//
//        // Wait for the game to start
//        waitForStart();
//        runtime.reset();
//
//        // Write all of our Autonomous objectives
//        while (opModeIsActive()) {
//            telemetry.addData("Status:", "Running");
//            telemetry.update();
//            encoderDrive(4);
//            go = new Thread(() -> {
//                encoderDrive(-4);
//            });
//            go.start();
//            mech = new Thread(() -> {
//                encoderLift(12);
//            });
//            mech.start();
//            while(go.isAlive()|| mech.isAlive());
//            robot.sleep(4000);
//        }
//    }
//
//    private void encoderDrive( double inches) {
//        int newTarget;
//        newTarget = robot.left.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//        robot.lf.setTargetPosition(newTarget);
//        robot.lb.setTargetPosition(newTarget);
//        robot.rf.setTargetPosition(newTarget);
//        robot.rb.setTargetPosition(newTarget);
//        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lb.setPower(.5);
//        robot.lf.setPower(.5);
//        robot.rb.setPower(.5);
//        robot.rf.setPower(.5);
//        while (opModeIsActive() && robot.left.isBusy()) {
//
//        }
//
//        // Stop all motion;
//        robot.halt();
//        robot.setEncoder();
//    }
//
//    private void encoderTurn(double degrees) {
//        int newTarget;
//        newTarget = robot.left.getCurrentPosition() + (int) ((degrees/360) * 2 * Math.PI * WHEEL_RADIUS * COUNTS_PER_INCH);
//        robot.lb.setTargetPosition(newTarget);
//        robot.lf.setTargetPosition(newTarget);
//        robot.rb.setTargetPosition(-newTarget);
//        robot.rf.setTargetPosition(-newTarget);
//        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lb.setPower(.5);
//        robot.lf.setPower(.5);
//        robot.rb.setPower(.5);
//        robot.rf.setPower(.5);
//        while (opModeIsActive() && robot.left.isBusy()) {
//
//        }
//
//        // Stop all motion;
//        robot.halt();
//        robot.setEncoder();
//
//
//    }
//
//    private void encoderLift(double inches) {
//        int newTarget;
//        newTarget = robot.lift.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//        robot.lift.setTargetPosition(newTarget);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(.5);
//
//        while (opModeIsActive() && robot.lift.isBusy()) {
//
//        }
//
//        // Stop all motion;
//        robot.halt();
//        robot.setEncoder();
//
//
//    }
//

}
