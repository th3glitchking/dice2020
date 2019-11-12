/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Preston on 9/19/18
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        //Initialize motors and their individual direction
        telemetry.addData("Status:", "Initializing");

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        robot.removeEncoder();


        // Wait for the game to start
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            //Control method for Driver 1
            new Thread(() -> {
                Drive(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.left_stick_x);
            }).start();

            //Control method for Driver 2
            new Thread(() -> {
                Mechanism(gamepad2.right_stick_y, gamepad2.right_trigger);
            }).start();

            telemetry.addData("Status:", "Running");
            telemetry.update();
        }
        robot.halt();
        telemetry.addData("Status:","Stopped");
        telemetry.update();
    }


    private void Drive(double drive, double back, double turn) {
        robot.lb.setPower(drive-back+turn);
        robot.lf.setPower(drive-back+turn);
        robot.rf.setPower(drive-back-turn);
        robot.rb.setPower(drive-back-turn);
    }

    private void Mechanism(double raise, double move){
        robot.arm.setPower(raise);
    }

}
