package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    // Later uses
    // Mecanum Drive
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor outakeArmMotor;

    // double reverse fourbar
    DcMotor leftDR4BMotor;
    DcMotor rightDR4BMotor;

    // coaxial virtual four bar
    Servo leftCV4BServo;
    Servo rightCV4BServo;
    Servo rotateIntakeServo;
    CRServo intakeServo;

    //Specimen
    Servo specimenServo;

    DistanceSensor distanceSensor;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        outakeArmMotor = hwMap.get(DcMotor.class, "outakeArmMotor");

        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        leftDR4BMotor = hwMap.get(DcMotor.class, "leftDR4BMotor");
        rightDR4BMotor = hwMap.get(DcMotor.class, "rightDR4BMotor");

        // MUST BE CHANGED LATER DRIVETRAIN WILL NOT WORK IF THIS IS NOT WORKING
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCV4BServo = hwMap.get(Servo.class, "leftCV4BServo");
        rightCV4BServo = hwMap.get(Servo.class, "rightCV4BServo");
        rotateIntakeServo = hwMap.get(Servo.class, "rotateIntakeServo");

        intakeServo = hwMap.get(CRServo.class, "intakeServo");

        leftDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDR4BMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        // o

        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }

    public double getDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("left DR4B motor position", leftDR4BMotor.getCurrentPosition());
        telemetry.addData("right DR4B motor position", rightDR4BMotor.getCurrentPosition());

        telemetry.addData("left CRVB motor position", leftCV4BServo.getPosition());
        telemetry.addData("right CR4B motor position", rightCV4BServo.getPosition());
        telemetry.addData("rotate intake servo position", rotateIntakeServo.getPosition());
        telemetry.addData("specimen servo position", specimenServo.getPosition());

        telemetry.addData("distance away from anything", getDistance(DistanceUnit.INCH));

        if (getDistance(DistanceUnit.INCH) <= 3) {
            telemetry.addLine("TOO CLOSE");
        } else if (getDistance(DistanceUnit.INCH) <= 6) {
            telemetry.addLine("GETTING CLOSE");
        }

        telemetry.update();
    }

}