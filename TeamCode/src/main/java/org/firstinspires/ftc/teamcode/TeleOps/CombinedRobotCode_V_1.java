package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Purple Spark Robot Code V1")
public class CombinedRobotCode_V_1 extends LinearOpMode {

    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

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
            DcMotorEx shooterR = (DcMotorEx) hardwareMap.get(DcMotor.class, "d1");
            DcMotorEx shooterL = (DcMotorEx) hardwareMap.get(DcMotor.class, "d2");

            shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //Motor direction configuration
            shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU object creation

        imu = hardwareMap.get(IMU.class, "imu");

            // Servo variable creation
            CRServo intakeServo1 =  hardwareMap.get(CRServo.class,"in1");; // First rubber band wheel
            CRServo intakeServo2 = hardwareMap.get(CRServo.class,"in2");; // Second rubber bad wheel
            CRServo chooserServoR = hardwareMap.get(CRServo.class,"a2");; // Right red diamond wheel
            CRServo chooserServoL = hardwareMap.get(CRServo.class,"a1");; // Left red diamond wheel
            CRServo regulationServo = hardwareMap.get(CRServo.class,"x");; // Black wheel

            // Servo direction configuration
            intakeServo1.setDirection(CRServo.Direction.REVERSE);
            intakeServo2.setDirection(CRServo.Direction.REVERSE);
            chooserServoR.setDirection(CRServo.Direction.FORWARD);
            chooserServoL.setDirection(CRServo.Direction.REVERSE);
            regulationServo.setDirection(CRServo.Direction.REVERSE);

            // Power variables
            double intakeServoPower;
            double chooserServoPowerR;
            double chooserServoPowerL;
            double shooterPower;
            double regulatorServoPower = 0;

        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

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
                if (gamepad2.left_trigger > 0.1){
                    intakeServoPower = 0.8;
                } else intakeServoPower = 0;


                //Alimentar bajo
                if(gamepad2.right_bumper) {
                    chooserServoPowerR = 0.8;
                    chooserServoPowerL = 0.8;
                    shooterPower = 1000;
                } else {
                    regulatorServoPower = 0;
                    shooterPower = 0;
                    chooserServoPowerL = 0;
                    chooserServoPowerR = 0;
                }

                //Alimentar alto
                // When the right trigger is pressed the shooter and control wheel turn on
                if (gamepad2.right_trigger > 0.1){
                    chooserServoPowerR = 0.8;
                    chooserServoPowerL = 0.8;
                    shooterPower = 1900;
                }

                if(gamepad2.y){
                    intakeServoPower = -0.8;
                }

                // When b is pressed the right chooser wheel turns
                if (gamepad2.b){
                    chooserServoPowerR = 0.8;
                    chooserServoPowerL = -0.8;
                    intakeServoPower = 0.8;

                }

                // When x is pressed the left chooser wheel turns
                if (gamepad2.x){
                    chooserServoPowerL = 0.8;
                    chooserServoPowerR = -0.8;
                    intakeServoPower = 0.8;
                }

                double shooterVelocityR = shooterR.getVelocity();
                double shooterVelocityL = shooterL.getVelocity();
                double soltureR = shooterR.getVelocity();
                double soltureL = shooterL.getVelocity();


                if(shooterVelocityL > 1870 && shooterVelocityR > 1870 && shooterVelocityL < 1930 && shooterVelocityR < 1930 ){
                    regulatorServoPower = 0.4;
                }

                if(shooterVelocityL > 970 && shooterVelocityR > 970 && shooterVelocityL < 1030 && shooterVelocityR < 1030 ){
                    regulatorServoPower = 0.4;
                }

                // Gives power to the intake
                intakeServo1.setPower(intakeServoPower);
                intakeServo2.setPower(intakeServoPower);

                // Gives power to the chooser wheels
                chooserServoL.setPower(chooserServoPowerL);
                chooserServoR.setPower(chooserServoPowerR);

                // Gives power to the shooter
                shooterR.setVelocity(shooterPower);
                shooterL.setVelocity(shooterPower);

                // Gives power to the control wheel
                regulationServo.setPower(regulatorServoPower);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Velocidad Chasis", currentPower);
                telemetry.addData("Velocidad Shooter R", shooterR.getVelocity());
                telemetry.addData("Velocidad Shooter L", shooterL.getVelocity());
                telemetry.addData("Poder Shooter", shooterPower);
                telemetry.addData("Heading", obtenerAngulo());
                telemetry.update();

            }
    }

    public double obtenerAngulo() {
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

}

