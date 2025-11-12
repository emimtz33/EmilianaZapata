package org.firstinspires.ftc.teamcode.Projectos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous
public class Autonomo extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wheels variable
        DcMotorEx leftDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "ch1");
        DcMotorEx rightDrive = (DcMotorEx) hardwareMap.get(DcMotor.class, "ch2");

        //Inverting one of the sides for a linear drive
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //Break or coast for motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Motor variable creation
        DcMotorEx shooterR = (DcMotorEx) hardwareMap.get(DcMotor.class, "d1");
        DcMotorEx shooterL = (DcMotorEx) hardwareMap.get(DcMotor.class, "d2");


        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Motor direction configuration
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo variable creation

        CRServo intakeServo1; // First rubber band wheel
        intakeServo1 = hardwareMap.get(CRServo.class, "in1");
        CRServo intakeServo2; // Second rubber bad wheel
        intakeServo2 = hardwareMap.get(CRServo.class, "in2");
        CRServo chooserServoR; // Right red diamond wheel
        chooserServoR = hardwareMap.get(CRServo.class, "a2");
        CRServo chooserServoL; // Left red diamond wheel
        chooserServoL = hardwareMap.get(CRServo.class, "a1");
        CRServo regulationServo; // Black wheel
        regulationServo = hardwareMap.get(CRServo.class, "x");

        // Servo direction configuration
        intakeServo1.setDirection(CRServo.Direction.REVERSE);
        intakeServo2.setDirection(CRServo.Direction.REVERSE);
        chooserServoR.setDirection(CRServo.Direction.FORWARD);
        chooserServoL.setDirection(CRServo.Direction.REVERSE);
        regulationServo.setDirection(CRServo.Direction.REVERSE);

        double intakeServoPower = 0;
        double chooserServoPowerR = 0;
        double chooserServoPowerL = 0;
        double shooterPower = 0;
        double regulatorServoPower = 0;

        double shooterVelocityR = shooterR.getVelocity();
        double shooterVelocityL = shooterL.getVelocity();
        double soltureR = shooterR.getVelocity();
        double soltureL = shooterL.getVelocity();
        double difernce = soltureL - soltureR;


        // Wait for the game to start (driver presses START)
        waitForStart();

/*        //Forward
        rightDrive.setVelocity(-1000);
        leftDrive.setVelocity(-1000);
        sleep(1000);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);

        //Turn
        rightDrive.setVelocity(1000);
        leftDrive.setVelocity(1000);
        sleep(500);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);

        //Backward
        rightDrive.setVelocity(-2000);
        leftDrive.setVelocity(-2000);
        sleep(1000);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);

*/
        //Forward
        rightDrive.setVelocity(-1000);
        leftDrive.setVelocity(-1000);
        sleep(1100);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);
        sleep(500);

        //Turn
        rightDrive.setVelocity(-1000);
        leftDrive.setVelocity(1000);
        sleep(350);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);
        sleep(500);

        //Forward
        rightDrive.setVelocity(-1000);
        leftDrive.setVelocity(-1000);
        sleep(500);
        rightDrive.setVelocity(0);
        leftDrive.setVelocity(0);

        chooserServoPowerR = 0.8;
        chooserServoPowerL = 0.8;
        shooterPower = 1250;
        chooserServoL.setPower(chooserServoPowerL);
        chooserServoR.setPower(chooserServoPowerR);
        shooterR.setVelocity(shooterPower);
        shooterL.setVelocity(shooterPower);
        sleep(500);
        regulatorServoPower = 0.8;
        regulationServo.setPower(regulatorServoPower);
        sleep(4000);
        intakeServoPower = 0.8;
        intakeServo1.setPower(intakeServoPower);
        intakeServo2.setPower(intakeServoPower);
        sleep(2000);
        intakeServoPower = 0;

        intakeServo1.setPower(intakeServoPower);
        intakeServo2.setPower(intakeServoPower);
        sleep(30000);
        requestOpModeStop();
    }
}
