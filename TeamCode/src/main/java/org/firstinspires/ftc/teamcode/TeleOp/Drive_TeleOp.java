package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;






@TeleOp(name="TeleOP")
public class Drive_TeleOp extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor FlMotor;
    public DcMotor BlMotor;
    public DcMotor FrMotor;
    public DcMotor BrMotor;
    public DcMotor WobbleFlipper;
    public DcMotor Intake;
    public DcMotor LeftShooter;
    public DcMotor RightShooter;


    public Servo WobbleGrabber;
    public CRServo ConveyorBelt;
    public Servo IntakeFlipper;


    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        // Retrieve the motor objects from the hardware map. These names come from the configuration in the robot controller.
        BlMotor = hardwareMap.get(DcMotor.class, "Backleft");
        FlMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        BrMotor = hardwareMap.get(DcMotor.class, "Backright");
        FrMotor = hardwareMap.get(DcMotor.class, "Frontright");
        WobbleFlipper = hardwareMap.get(DcMotor.class, "WobbleFlipper");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");


        WobbleGrabber = hardwareMap.get(Servo.class, "WobbleGrabber");
        ConveyorBelt = hardwareMap.get(CRServo.class, "Convey");
        IntakeFlipper = hardwareMap.get(Servo.class, "Flipper");


//  SET POSITION OF OUR SERVOS EXAMPLE BELOW
//        wideGrabber.setPosition(1);


        // Since one motor is reversed in relation to the other, we must reverse the motor on the right so positive powers mean forward.
        BlMotor.setDirection(DcMotor.Direction.FORWARD);
        FlMotor.setDirection(DcMotor.Direction.FORWARD);
        BrMotor.setDirection(DcMotor.Direction.REVERSE);
        FrMotor.setDirection(DcMotor.Direction.REVERSE);

        RightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        BlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WobbleFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FlMotor.setPower(0);
        FrMotor.setPower(0);
        BlMotor.setPower(0);
        BrMotor.setPower(0);
        WobbleFlipper.setPower(0);
        ConveyorBelt.setPower(0);
        LeftShooter.setPower(0);
        RightShooter.setPower(0);






        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void loop() {

        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

//        BlMotor.setPower(secondaryDiagonalSpeed - rotation);
//        FrMotor.setPower(secondaryDiagonalSpeed + rotation);
//        FlMotor.setPower(primaryDiagonalSpeed - rotation);
//        BrMotor.setPower(primaryDiagonalSpeed + rotation);
        BlMotor.setPower(secondaryDiagonalSpeed + rotation);
        FrMotor.setPower(secondaryDiagonalSpeed - rotation);
        FlMotor.setPower(primaryDiagonalSpeed + rotation);
        BrMotor.setPower(primaryDiagonalSpeed - rotation);

//        telemetry.addData("Primary:", primaryDiagonalSpeed);
//        telemetry.addData("Secondary:", secondaryDiagonalSpeed);
//        telemetry.update();


        //Intake and Conveyor belt Code
        if (gamepad2.left_trigger > 0.3) {
            Intake.setPower(0.9);
            ConveyorBelt.setPower(0.8);
        } else if (gamepad2.right_trigger > 0.3) {
            Intake.setPower(0.9);
            ConveyorBelt.setPower(0.8);
        }else{
            Intake.setPower(0);
            ConveyorBelt.setPower(0);
        }

        if (gamepad2.left_bumper) {
            Intake.setPower(-0.9);
            ConveyorBelt.setPower(-0.8);
        }else if (gamepad2.right_bumper) {
            Intake.setPower(-0.9);
            ConveyorBelt.setPower(-0.8);
        }else {
            Intake.setPower(0);
            ConveyorBelt.setPower(0);
        }

        //Shooter code

        if (gamepad2.left_stick_y >= 0.6) {
            LeftShooter.setPower(-0.7);
            RightShooter.setPower(-0.7);
        }else if (gamepad2.left_stick_y <= -0.6) {
            LeftShooter.setPower(0.7);
            RightShooter.setPower(0.7);
        }else{
                LeftShooter.setPower(0);
                RightShooter.setPower(0);
            }


        telemetry.addData("Left shooter:", gamepad2.left_stick_y);
        telemetry.addData("Right shooter:", gamepad2.left_stick_y);
        telemetry.update();


        //wobble Flipper
        if (gamepad2.dpad_up) {
            WobbleFlipper.setPower(1);
        } else {
            WobbleFlipper.setPower(0);
        }

        if (gamepad2.dpad_down) {
            WobbleFlipper.setPower(-1);
        } else {
            WobbleFlipper.setPower(0);
        }
        //Wobble Grabber
        if (gamepad2.dpad_left) {
            WobbleGrabber.setPosition(1);
        } else if (gamepad2.dpad_right) {
            WobbleGrabber.setPosition(0);
        }

        if (gamepad2.b) {
            IntakeFlipper.setPosition(0.9);
        } else if (gamepad2.x){
            IntakeFlipper.setPosition(0);
        }


        if (gamepad1.right_trigger > 0.3) {

            SlowMode();

        }



    }

    public void SlowMode () {
        double speed = Math.sqrt(2) * Math.pow(Math.pow(gamepad1.left_stick_x, 4) + Math.pow(gamepad1.left_stick_y, 4), 0.5);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rotation = Math.signum(gamepad1.right_stick_x) * Math.pow(gamepad1.right_stick_x, 2);

        float primaryDiagonalSpeed = (float) (speed * Math.sin(angle - (Math.PI / 4.0)));
        float secondaryDiagonalSpeed = (float) (speed * Math.cos(angle - (Math.PI / 4.0)));

//        BlMotor.setPower(secondaryDiagonalSpeed - rotation);
//        FrMotor.setPower(secondaryDiagonalSpeed + rotation);
//        FlMotor.setPower(primaryDiagonalSpeed - rotation);
//        BrMotor.setPower(primaryDiagonalSpeed + rotation);
        BlMotor.setPower(0.5 * secondaryDiagonalSpeed + rotation);
        FrMotor.setPower(0.5 * secondaryDiagonalSpeed - rotation);
        FlMotor.setPower(0.5 * primaryDiagonalSpeed + rotation);
        BrMotor.setPower(0.5 * primaryDiagonalSpeed - rotation);

    }
}




