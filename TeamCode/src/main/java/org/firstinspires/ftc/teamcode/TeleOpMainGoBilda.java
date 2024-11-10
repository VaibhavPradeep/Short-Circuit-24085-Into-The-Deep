
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOpMainGoBilda")
public class TeleOpMainGoBilda extends OpMode {
    MecanumDriveCircuit mecanumDrive = new MecanumDriveCircuit(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles;
    double speed = 0.8;
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    // CHANGE LATER yy
    final double SPECIMEN_COLLECT = 1.0;
    final double SPECIMEN_HOLD = 0;

    private RobotHardware robotHw = new RobotHardware();

    double DR4B_SPEED = 0.6;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor armMotor;
    CRServo intakeServo;
    Servo wristServo;
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;
    Servo specimenServo;

    public void moveIntakeServo () {
        if (gamepad1.a) {
            intakeServo.setPower(INTAKE_COLLECT);
            telemetry.addLine("yuh");
        }
        else if (gamepad1.x) {
            intakeServo.setPower(INTAKE_OFF);
        }
        else if (gamepad1.b) {
            intakeServo.setPower(INTAKE_DEPOSIT);
        }
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
        armMotor.setPower(speed);

        if (armMotor.getCurrentPosition() == armPos) {
            armMotor.setPower(0);
        }

    }

    public void moveDR4BMotors () {
        int leftDR4BMotorPos = leftDR4BMotor.getCurrentPosition();
        int rightDR4BMotorPos = rightDR4BMotor.getCurrentPosition();

        if(gamepad2.dpad_up) {
            leftDR4BMotorPos += 50;
            rightDR4BMotorPos += 50;
        }
        else if(gamepad2.dpad_down) {
            leftDR4BMotorPos -= 50;
            rightDR4BMotorPos -= 50;
        }

        leftDR4BMotor.setTargetPosition(leftDR4BMotorPos);
        rightDR4BMotor.setTargetPosition(rightDR4BMotorPos);
        leftDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDR4BMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDR4BMotor.setPower(DR4B_SPEED);
        rightDR4BMotor.setPower(DR4B_SPEED);

        if (leftDR4BMotor.getCurrentPosition() == leftDR4BMotorPos && rightDR4BMotor.getCurrentPosition() == rightDR4BMotorPos) {
            leftDR4BMotor.setPower(0);
            rightDR4BMotor.setPower(0);
        }

    }

    public void moveSpecimenServo() {
        if (gamepad2.left_bumper) {
            specimenServo.setPosition(SPECIMEN_COLLECT);
            telemetry.addLine("yuh");
        }
        else if (gamepad2.right_bumper) {
            specimenServo.setPosition(SPECIMEN_HOLD);
        }

    }

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

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        leftDR4BMotor = hardwareMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hardwareMap.get(DcMotor.class, "rightDR4BMotor");

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        wristServo = hardwareMap.get(Servo.class, "wristServo");
        specimenServo = hardwareMap.get(Servo.class, "specimenServo");

        // MUST BE CHANGED LATER DRIVETRAIN WILL NOT WORK IF THIS IS NOT WORKING
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(INTAKE_OFF);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        // o

        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        moveIntakeServo();

        moveDR4BMotors();

        moveArmMotor();

        mecanumDrive.driveMecanum();

        moveSpecimenServo();

        moveWristServo();

    }
}
