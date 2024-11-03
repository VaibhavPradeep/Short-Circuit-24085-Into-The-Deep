package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ScrimBotTeleOp")
public class ScrimBotTeleOp extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    //DcMotor armSlide;
    DcMotor armMotor;
    CRServo intakeServo;
    Servo wristServo;
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    int[] maxPositions = {2100}; // potentially something else

    int[] minPositions = {0};
    double slideSpeed = 0.6;

    public void driveMechanum(double left_y, double left_x, double right_x){
        double maxPower = Math.max(Math.abs(left_y) + Math.abs(left_x) + Math.abs(right_x), 1);
        frontLeft.setPower((left_y - left_x - right_x) / maxPower);
        frontRight.setPower((left_y + left_x + right_x) / maxPower);
        backLeft.setPower((left_y + left_x - right_x) / maxPower);
        backRight.setPower((left_y - left_x + right_x) / maxPower);
    }

    public void moveArmMotor () {
        int armPos = armMotor.getCurrentPosition();

        if(gamepad2.dpad_up) {
            armPos += 50;
        }
        else if(gamepad2.dpad_down) {
            armPos -= 50;
        }

        armMotor.setTargetPosition(armPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(slideSpeed);

        if (armMotor.getCurrentPosition() == armPos) {
            armMotor.setPower(0);
        }

    }

    /* public void moveArmSlide () {
       int slidePos = armSlide.getCurrentPosition();

       if(gamepad2.y && slidePos <= maxPositions[0]) {
            slidePos += 3000;
        }
        else if(gamepad2.a && slidePos >= minPositions[0]) {
            slidePos -= 3000;
        }

        armSlide.setTargetPosition(slidePos);
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setPower(slideSpeed);

        if (armSlide.getCurrentPosition() == slidePos) {
            armSlide.setPower(0);
        }
    } */

    public void moveWristServo() {
        double servoPosition = wristServo.getPosition();
        if (gamepad2.left_bumper) {
            servoPosition = Math.min(1.0, servoPosition + 0.005);
            wristServo.setPosition(servoPosition);
        } else if (gamepad2.right_bumper) {
            servoPosition = Math.max(0.0, servoPosition - 0.005);
            wristServo.setPosition(servoPosition);
        }
    }

    public void moveIntakeServo () {
        if (gamepad2.a) {
            intakeServo.setPower(INTAKE_COLLECT);
            telemetry.addLine("yuh");
        }
        else if (gamepad2.x) {
            intakeServo.setPower(INTAKE_OFF);
        }
        else if (gamepad2.b) {
            intakeServo.setPower(INTAKE_DEPOSIT);
        }
    }

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //armSlide = hardwareMap.get(DcMotor.class, "armSlide");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// jk
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(INTAKE_OFF);
        //armSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

    }

    @Override
    public void loop() {
        driveMechanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        moveIntakeServo();
        moveWristServo();
        moveArmMotor();
        //moveArmSlide();

        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        //telemetry.addData("Target Position", armSlide.getTargetPosition());
        //telemetry.addData("Current Position", armSlide.getCurrentPosition());
        telemetry.update();
        telemetry.update();
        // oi oko ojoj iuift
    }
}
