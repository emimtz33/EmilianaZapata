package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;


@TeleOp(name = "Purple Spark Robot Code OBSOLETE")

public class PurpleSparkCode extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        //Telemetry initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wheels variable
        DcMotor leftDrive = hardwareMap.get(DcMotor.class, "ch1");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "ch2");

        //Inverting one of the sides for a linear drive
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //Break or coast for motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wheel speed configuration
        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        double lowPower = 0.3;
        double highPower = 0.8;
        double currentPower = lowPower;

        //Motor variable creation
        DcMotor shooterR = hardwareMap.get(DcMotor.class,"d1");
        DcMotor shooterL = hardwareMap.get(DcMotor.class,"d2");

        //Motor direction configuration
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servo variable creation

        CRServo intakeServo1; // First rubber band wheel
        intakeServo1 =  hardwareMap.get(CRServo.class,"in1");
        CRServo intakeServo2; // Second rubber bad wheel
        intakeServo2 = hardwareMap.get(CRServo.class,"in2");
        CRServo chooserServoR; // Right red diamond wheel
        chooserServoR = hardwareMap.get(CRServo.class,"a2");
        CRServo chooserServoL; // Left red diamond wheel
        chooserServoL = hardwareMap.get(CRServo.class,"a1");
        CRServo regulationServo; // Black wheel
        regulationServo = hardwareMap.get(CRServo.class,"x");

        // Servo direction configuration
        intakeServo1.setDirection(CRServo.Direction.REVERSE);
        intakeServo2.setDirection(CRServo.Direction.REVERSE);
        chooserServoR.setDirection(CRServo.Direction.FORWARD);
        chooserServoL.setDirection(CRServo.Direction.REVERSE);
        regulationServo.setDirection(CRServo.Direction.REVERSE);

        // Power variables
        double intakeServoPower = 0;
        double chooserServoPowerR = 0;
        double chooserServoPowerL = 0;
        double shooterPower = 0;
        double regulatorServoPower = 0;

        waitForStart();
        runtime.reset();
        // Waits for the robot to start

        while(opModeIsActive()){

            // Robot movement
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Fast/Slow mode change when dpad right is pressed
            if(gamepadRateLimit.hasExpired() && gamepad1.dpad_right){
                currentPower = (currentPower == lowPower) ? highPower : lowPower;
                gamepadRateLimit.reset();
            }

            //Drivetrain configuration for arcade drive
            double leftPower = Range.clip(drive - turn, -currentPower, currentPower); //limits the range of the motor
            double rightPower = Range.clip(drive + turn, -currentPower, currentPower);

            //Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // When left trigger is pressed the intake turns on
            if (gamepad1.left_trigger > 0.1){
                intakeServoPower = 0.8;
            } else intakeServoPower = 0;

            // When the right trigger is pressed the shooter and control wheel turn on
            if (gamepad1.right_trigger > 0.1){
                shooterPower = 0.8;

            }else {regulatorServoPower = 0;
                    shooterPower = 0;}

            if(gamepad1.y){
                intakeServoPower = -0.8;
            }

            // When b is pressed the right chooser wheel turns
            if (gamepad1.b){
                chooserServoPowerR = 0.8;
                chooserServoPowerL = -0.8;
                intakeServoPower = 0.8;
            }else {chooserServoPowerR = 0;
                    chooserServoPowerL = 0;}

            // When x is pressed the left chooser wheel turns
            if (gamepad1.x){
                chooserServoPowerL = 0.8;
                chooserServoPowerR = -0.8;
                intakeServoPower = 0.8;
            }


            // Gives power to the intake
            intakeServo1.setPower(intakeServoPower);
            intakeServo2.setPower(intakeServoPower);

            // Gives power to the chooser wheels
            chooserServoL.setPower(chooserServoPowerL);
            chooserServoR.setPower(chooserServoPowerR);

            // Gives power to the shooter
            shooterR.setPower(shooterPower);
            shooterL.setPower(shooterPower);

            // Gives power to the control wheel
            regulationServo.setPower(regulatorServoPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocidad Chasis", currentPower);
            telemetry.update();
            
            if (gamepad1.right_trigger > 0.4){
                sleep(1000);
            if (shooterPower == 0.8){
                regulatorServoPower = 0.8;
            }}
        }
    }

}
