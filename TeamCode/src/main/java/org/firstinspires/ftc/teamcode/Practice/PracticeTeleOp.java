package org.firstinspires.ftc.teamcode.Practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Practice")
public class PracticeTeleOp extends OpMode {

    public DcMotor Frontleft;
    private DcMotor Frontright;
    private DcMotor Backleft;
    private DcMotor Backright;
    private DcMotor RShooter;
    private DcMotor LShooter;
    private DcMotor Intake;
    private DcMotor Wobble;

    private Servo Wobblegrabber;
    private Servo intakepusher;
    private CRServo mover;

    @Override
    public void init()
    {
        Frontleft = hardwareMap.get(DcMotor.class  "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class);
        Backleft = hardwareMap.get(DcMotor.class);
        Backright = hardwareMap.get(DcMotor.class);
        RShooter = hardwareMap.get(DcMotor.class);
        LShooter = hardwareMap.get(DcMotor.class);
        Intake = hardwareMap.get(DcMotor.class);
        Wobble = hardwareMap.get(DcMotor.class);

        Wobblegrabber = hardwareMap.get(Servo.class);
        intakepusher = hardwareMap.get(Servo.class);
        mover = hardwareMap.get(CRServo.class);


        Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        Backright.setDirection(DcMotorSimple.Direction.FORWARD);
        Backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        RShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        LShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Frontleft.setPower(0);
        Frontright.setPower(0);
        Backright.setPower(0);
        Backleft.setPower(0);
        Wobble.setPower(0);
        mover.setPower(0);
        RShooter.setPower(0);
        LShooter.setPower(0);

    }

    @Override
    public void loop()
    {
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(-gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

        Frontright.setPower(secondaryDiagonalSpeed - rotation);
        Frontleft.setPower(secondaryDiagonalSpeed + rotation);
        Backright.setPower(primaryDiagonalSpeed - rotation);
        Backleft.setPower(primaryDiagonalSpeed + rotation);


        //Intake + convator
        if (gamepad2.left_stick_y >= 0.3) {
            Intake.setPower(0.9);
            mover.setPower(0.8);
        }else if (gamepad2.left_stick_y <= -0.3){
            Intake.setPower(-0.9);
            mover.setPower(-0.8);
        }else{
            Intake.setPower(0);
            mover.setPower(0);
        }

        //Shooter
        if (gamepad2.right_stick_y >= 0.6){
            LShooter.setPower(-0.8);
            RShooter.setPower(-0.8);
        }else if (gamepad2.right_stick_y <= -0.6){
            LShooter.setPower(0.8);
            RShooter.setPower(0.8);
        }else{
            LShooter.setPower(0);
            RShooter.setPower(0);
        }

        //Wobble
        if (gamepad2.dpad_up) {
            Wobble.setPower(1);
        } else if (gamepad2.dpad_down) {
            Wobble.setPower(-1);
        }else {
            Wobble.setPower(0);
        }

        //Wobblegrabber
        if (gamepad2.dpad_left) {
            Wobblegrabber.setPosition(1);
        } else if (gamepad2.dpad_right) {
            Wobblegrabber.setPosition(0);
        }

        //Intakepusher
        if (gamepad2.b) {
            intakepusher.setPosition(0.9);
        } else if (gamepad2.x){
            intakepusher.setPosition(0);
        }
    }
}
