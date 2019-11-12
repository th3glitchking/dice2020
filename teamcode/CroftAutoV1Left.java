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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Croft Auto Left", group="Iterative Opmode")
@Disabled
public class CroftAutoV1Left extends OpMode {
    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;

    private long turnTime;

    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark;
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;


    public void init()
    {
        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Claw = hardwareMap.servo.get("Claw");
        ArmRotation = hardwareMap.dcMotor.get("ArmRotation");
        turnTime = 250;
    }

    public void start()
    {
        //ideal if camera code works
        //cut out if statement and place block in center if camera code not ready
        //need to write code if starting from the right, this code only works for left side
        //code starts with robot facing allies and with a block in the claw


        //move("forward", 200);
        //lift("rotateF", 200);//set arm up straight
        //move("right", 375);//position phone to read picture
        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AUab8lD/////AAAAGfdEGvojlULkpQTDLe8zdwREFmZqTFDUvuLi1b6FRcufb+FreEylImIJaeb5PJicszzZ7xRUFfpxp+uo1Elpr0fZPguNlkasbrN3ptWrr1wb7GqCfblzW1U/yYge9rOP5/Xc5UsjwpHI1xx42x78GS8ARqVCuHrsofoIx7y2sUaWfwZ5iK6wGJNl/fhArJ4PhG17kM2D3XFzKlcW+htqK4BWdIIeYqjbe76vvGGq6OJjqFMGM4ny8jOGr56GjtKNxHewsDkbm7sb8J9QVqZNUG0kHa5DBL9yzpaB7xLYZJNbN7gwDiLhWjGM58y61VP2pw06Zd2ZMCEETKhWHIAjZ3OWaS/CQSH6K46jXnzYOH/G"; // need access key
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTrackables.activate();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        //move("left", 375);
        //if (vuMark == RelicRecoveryVuMark.LEFT) {
            //move("forward", 520);//furthest distance
        //}
        //else if (vuMark == RelicRecoveryVuMark.CENTER) {
            //move("forward", 510);//middle distance
        //}
        //else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            //move("forward", 500);//shortest distance
        //}
        //move("right", turnTime);//turn
        //move("forward", 200);
        //Claw.setPosition(0);//place block


        cameraTest();//will rotate(right then left) to pick up image and then turn move based on image read
    }

    public void loop() {

    }

    public void move(String direction, long time)//left, right, forward, back
    {
        if(direction.equals("left"))
        {
            RWheel.setPower(-.5);
            LWheel.setPower(-.5);
        }
        else if(direction.equals("forward"))
        {
            RWheel.setPower(-.5);
            LWheel.setPower(.5);
        }
        else if(direction.equals("right"))
        {
            RWheel.setPower(.5);
            LWheel.setPower(.5);
        }
        else if(direction.equals("back"))
        {
            RWheel.setPower(.5);
            LWheel.setPower(-.5);
        }
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        RWheel.setPower(0);
        LWheel.setPower(0);
    }

    public void lift(String component, long time)
    {
        if(component.equals("rotateF"))
        {
            ArmRotation.setPower(.1);
        }
        if(component.equals("rotateB"))
        {
            ArmRotation.setPower(.1);
        }
        else if(component.equals("up"))
        {
            ArmHeight.setPower(.1);
        }
        else if(component.equals("down"))
        {
            ArmHeight.setPower(.1);
        }
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        ArmRotation.setPower(0);
        ArmHeight.setPower(0);
    }
    public void cameraTest()//test cameras
    {
        move("right", 375);//position phone to read picture
        move("left", 375);
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            move("left", 520);//furthest distance
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER) {
            move("forward", 510);//middle distance
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            move("right", 500);//shortest distance
        }
    }
}