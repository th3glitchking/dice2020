package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Croft TeleOp", group="Linear Opmode")
public class CroftTele extends OpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;

    private double leftP;
    private double rightP;
    private double armRotationP;
    private double armHeightP;
    private double clawP;
    private double drivePower;

    public void init()
    {
        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Claw = hardwareMap.servo.get("Claw");
        ArmRotation = hardwareMap.dcMotor.get("ArmRotation");
    }
    public void loop() {

        leftP = Range.clip(gamepad1.left_stick_y, -1, 1);
        rightP = Range.clip(gamepad1.right_stick_y, -1, 1);
        armRotationP = Range.clip(gamepad2.right_stick_y, -1, 1);
        armHeightP = Range.clip(gamepad2.left_stick_y, -1, 1);
        clawP = Range.clip(gamepad2.right_trigger, 0, 1);

        drivePower = Range.clip(gamepad1.right_trigger, 0, 1);

        LWheel.setPower(-leftP / ((3 * drivePower) + 1));
        RWheel.setPower(rightP / ((3 * drivePower) + 1));
        ArmRotation.setPower(armRotationP / 10);
        ArmHeight.setPower(armHeightP / 10);
        Claw.setPosition(clawP);//is 0 or 1 open?

    }
}