package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ScrimBotAutoMethods {
    ElapsedTime timer = new ElapsedTime();

    TeleOp opMode;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor armSlide;
    DcMotor armMotor;
    CRServo intakeServo;
    Servo wristServo;

    final double WHEEL_DIAMETER = 4.09;
    final double TICKS_PER_WHEEL_REVOLUTION = 384.5;
    final double INCHES_PER_WHEEL_REVOLUTION = 12.8632;
    final double TICKS_PER_INCH = (TICKS_PER_WHEEL_REVOLUTION) / (WHEEL_DIAMETER * Math.PI);
    final double DRIVE_GEAR_REDUCTION = 1.0; // just one so not needed in equation

    // Gyro
    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles;

    // Movement parameters

    // Motor power values
    double drivePower = 0.5; // Power for driving
    double armPower = 1.0; // Power for the arm


    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // find the name on driver hub for this
        armSlide = hwMap.get(DcMotor.class, "arm slide");

        // get some names on the driver hub for this
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
        wristServo = hwMap.get(Servo.class, "sidetoSideServo");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Init Gyro
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        imu = hwMap.get(IMU.class, "imu");
        angles = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorPowers(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void stopMotors() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void setTargetPosition(int ticks) {
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
    }

    public void setZeroPowerModeAsBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake(double time, String direction) {
        timer.reset();
        if (direction == "IN") {
            intakeServo.setPower(1.0);
        } else if (direction == "OUT") {
            intakeServo.setPower(-1.0);
        }
        // change time
        if (timer.milliseconds() >= time) {
            intakeServo.setPower(0.0);
        }
    }

    public void moveArmMotor(double power, int encoder, String direction) {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(power);
        // change encoder value
        if (direction == "FORWARD") {
            armMotor.setTargetPosition(encoder);
        }
        else if (direction == "REVERSE") {
            armMotor.setTargetPosition(-encoder);
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendArmSlide(double power, int encoder, String direction) {
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setPower(power);
        // change encoder value
        if (direction == "FORWARD") {
            armSlide.setTargetPosition(encoder);
        }
        else if (direction == "REVERSE") {
            armSlide.setTargetPosition(encoder);
        }
        armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlide.setPower(0.0);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void drive(double inches, String direction) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        resetEncoders();
        if (direction == "FORWARD") {
            setTargetPosition(targetTicks);
        } else if (direction == "BACKWARD"){
            setTargetPosition(-targetTicks);
        }
        setMotorPowers(drivePower);
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Wait until all motors reach their target position
        }
        stopMotors();
    }

    public void strafe(double inches, String direction) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        resetEncoders();
        if (direction == "LEFT") {
            frontLeft.setTargetPosition(-targetTicks);
            frontRight.setTargetPosition(targetTicks);
            backLeft.setTargetPosition(targetTicks);
            backRight.setTargetPosition(-targetTicks);
        } else if (direction == "RIGHT") {
            frontLeft.setTargetPosition(targetTicks);
            frontRight.setTargetPosition(-targetTicks);
            backLeft.setTargetPosition(-targetTicks);
            backRight.setTargetPosition(targetTicks);
        }

        setMotorPowers(drivePower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            // Wait until all motors reach their target position
        }
    }

    public void releaseSampleWristServo() {
        timer.reset();
        // Implement logic to release the preloaded sample, e.g., moving an arm or servo
        // This is just an example and may vary based on your robot's design
        wristServo.setPosition(1.0); // Example position to release
        if (timer.milliseconds() >= 2000) { // it can be changed
            wristServo.setPosition(0.0); // Reset position
        }
    }

    // used for turning
    public double getAngleDifference(double angle1, double angle2) {

        return (angle1 - angle2 + 540) % 360 - 180;

    }

    public void turn(double power, double angle, String direction) {
        angles = imu.getRobotYawPitchRollAngles();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int directionMultiplier = -1;

        if (direction.equals("CLOSEST")) {

            if (getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {
                power *= -1;
                directionMultiplier *= -1;
            }

        } else if (direction.equals("LEFT")) {
            power *= -1;
            directionMultiplier *= -1;
        }

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && directionMultiplier * getAngleDifference(angles.getYaw(AngleUnit.DEGREES), angle) < 0) {

        }

        stopMotors();

        resetEncoders();
    }

    public void updateAutoTelemetry(Telemetry telemetry) {
        int frontLeftPosition = frontLeft.getCurrentPosition();
        int frontRightPosition = frontRight.getCurrentPosition();
        int backLeftPosition = backLeft.getCurrentPosition();
        int backRightPosition = backRight.getCurrentPosition();
        int armMotorPosition = armMotor.getCurrentPosition();
        int armSlidePosition = armSlide.getCurrentPosition();
        telemetry.addData(String.valueOf(frontLeftPosition), "front left position");
        telemetry.addData(String.valueOf(frontRightPosition), "front right position");
        telemetry.addData(String.valueOf(backLeftPosition), "back left position");
        telemetry.addData(String.valueOf(backRightPosition), "back right position");
        telemetry.addData(String.valueOf(armMotorPosition), "arm motor position");
        telemetry.addData(String.valueOf(armSlidePosition), "arm slide position");

        int frontLeftTargetPosition = frontLeft.getTargetPosition();
        int frontRightTargetPosition = frontRight.getTargetPosition();
        int backLeftTargetPosition = backLeft.getTargetPosition();
        int backRightTargetPosition = backRight.getTargetPosition();
        int armMotorTargetPosition = armMotor.getTargetPosition();
        int armSlideTargetPosition = armSlide.getTargetPosition();
        telemetry.addData(String.valueOf(frontLeftTargetPosition), "front left target position");
        telemetry.addData(String.valueOf(frontRightTargetPosition), "front right target position");
        telemetry.addData(String.valueOf(backLeftTargetPosition), "back left target position");
        telemetry.addData(String.valueOf(backRightTargetPosition), "back right target position");
        telemetry.addData(String.valueOf(armMotorTargetPosition), "arm motor target position");
        telemetry.addData(String.valueOf(armSlideTargetPosition), "arm slide target position");

        double frontLeftPower = frontLeft.getPower();
        double frontRightPower = frontRight.getPower();
        double backLeftPower = backLeft.getPower();
        double backRightPower = backRight.getPower();
        telemetry.addData(String.valueOf(frontLeftPower), "front left power");
        telemetry.addData(String.valueOf(frontRightPower), "front right power");
        telemetry.addData(String.valueOf(backLeftPower), "back left power");
        telemetry.addData(String.valueOf(backRightPower), "back right power");

        double wristServoPosition = wristServo.getPosition();
        telemetry.addData(String.valueOf(wristServoPosition), "side to side servo position");

    }

}
