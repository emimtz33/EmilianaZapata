package org.firstinspires.ftc.teamcode.TeleOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BlueSpark RobotCode")
public class BlueSpark_RobotCode extends LinearOpMode {

    public IMU imu;

    @Override
    public void runOpMode(){

        telemetry.addData("Estado", "Inicializado correctamente");
        telemetry.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Inicializar motores
        DcMotorEx leftFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFront");
        DcMotorEx rightFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFront");
        DcMotorEx leftBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBack");
        DcMotorEx rightBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBack");

        //IMU object creation
        imu = hardwareMap.get(IMU.class, "imu");

        //Initialize the hardware variables
        DcMotorEx ShooterR = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooterR");
        DcMotorEx ShooterL = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooterL");
        DcMotor Intake = hardwareMap.get(DcMotor.class, "intake1");

        // Configure robot orientation
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // Function orientations
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Servo object creation
        CRServo servoIntake;
        servoIntake = hardwareMap.get(CRServo.class, "seervo");


        // IMU object creation
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
        );

        //Intake toogle
        boolean currentStatus = false;
        boolean previousStatus;


        waitForStart();

        while(opModeIsActive()) {

            // DRIVER CODE

            //Joystick variable creation
            double drive = gamepad1.left_stick_y; //Eje Y invertido
            double strafe = -(gamepad1.left_stick_x);
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;



            if (gamepad1.dpad_down) {
                rightBack.setVelocity(1500);
            }

            if (gamepad1.dpad_up) {
                leftFront.setVelocity(500);
            }

            boolean modoLento = gamepad1.circle;  //Define Slow mode
            double maxPower = modoLento ? 500 : 1500;    //Motor limitations

            //Wheel variable creation
            double lFPower = Range.clip((drive + turn + strafe), -maxPower, maxPower);
            double rFPower = Range.clip((drive - turn - strafe), -maxPower, maxPower);
            double lBPower = Range.clip((drive + turn - strafe), -maxPower, maxPower);
            double rBPower = Range.clip((drive - turn + strafe), -maxPower, maxPower);



            //OPERATOR CODE

            //Intake and shooter variables
            double powerI;
            double powerSRight;
            double powerSLeft;
            double powerServo;



            double shooterLvelocity = ShooterL.getVelocity();
            double shooterRvelocity = ShooterR.getVelocity();


            /*boolean changed = false; //Outside of loop()
            if(gamepad1.a && !changed) {

                if(servo.getPosition() == 0) {
                servo.setPosition(1);
                } else {
                servo.setPosition(0);
                changed = true;
            } else if(!gamepad1.a) {
            changed = false;
            }
             */

            //Shooter function
            if (gamepad2.right_trigger > 0.1 || gamepad1.right_bumper) {
                powerSRight = 2700;
                powerSLeft = 2500;
            } else {
                powerSRight = 0;
                powerSLeft = 0;
            }

            if(gamepad2.right_bumper){
                powerSRight = 2300;
                powerSLeft = 2100;
            }

            //Intake function
            if(gamepad2.left_trigger > 0.1){
                powerI = 0.8;
            }else {
                powerI = 0;
            }



            if(gamepad2.circle && !currentStatus) {

            }

            //Inverse orientation intake
            if (gamepad2.triangle) {
                powerI = -0.8;
            }



            //Servo condition creation
            if(shooterLvelocity > 1000 && shooterRvelocity > 1000 && shooterRvelocity < 3000 && shooterLvelocity < 3000){
                powerServo = 0.8;
                servoIntake.setPower(powerServo);
            } else if (gamepad2.square) {
                powerServo = 0.8;
            } else {
                powerServo = 0.0;
            }
//Math.abs(targetVelocity - velocity) >= kEpsilon

            //Give power to intake and shooter motors (and servo)
            Intake.setPower(powerI);
            ShooterR.setVelocity(powerSRight);
            ShooterL.setVelocity(powerSLeft);
            servoIntake.setPower(powerServo);

            //Give power to the motors
            leftFront.setVelocity(lFPower * 1500);
            rightFront.setVelocity(rFPower * 1500);
            leftBack.setVelocity(lBPower * 1500);
            rightBack.setVelocity(rBPower * 1500);

            if(gamepad2.dpad_left){
                gamepad1.rumbleBlips(1);
            }

            if(gamepad1.dpad_left){
                gamepad2.rumbleBlips(1);
            }


            //Telemetry
            telemetry.addLine("Operator Telemetry");
            telemetry.addData("Velocidad Intake: ", Intake.getPower());
            telemetry.addData("Servo Power", servoIntake.getPower());
            telemetry.addData("Velocidad Shooter Izquierdo: ", shooterLvelocity);
            telemetry.addData("Velocidad Shooter Derecho", shooterRvelocity);
            telemetry.addData("Rotacion de HEX1: ", Intake.getDirection());
            telemetry.addLine();
            telemetry.addLine("Driver Telemetry");
            telemetry.addData("Modo Lento", modoLento ? "ACTIVADO" : "Desactivado");
            telemetry.addData("FI Velocidad",leftFront.getVelocity());
            telemetry.addData("FD Velocidad", rightFront.getVelocity());
            telemetry.addData("AI Velocidad", leftBack.getVelocity());
            telemetry.addData("AD Velocidad", rightBack.getVelocity());
            telemetry.addData("IMU Heading: ", telemetryImu());
            telemetry.update();

            //Dashboard Telemetria
            dashboardTelemetry.addLine("Operator Telemetry");
            dashboardTelemetry.addData("Velocidad Intake: ", powerI);
            dashboardTelemetry.addData("Servo Power", powerServo);
            dashboardTelemetry.addData("Velocidad Shooter Izquierdo: ", shooterLvelocity);
            dashboardTelemetry.addData("Velocidad Shooter Derecho", shooterRvelocity);
            dashboardTelemetry.addData("Rotacion de HEX1: ", Intake.getDirection());
            dashboardTelemetry.addLine();
            dashboardTelemetry.addLine("Driver Telemetry");
            dashboardTelemetry.addData("Modo Lento", modoLento ? "ACTIVADO" : "Desactivado");
            dashboardTelemetry.addData("FI Power", lFPower);
            dashboardTelemetry.addData("FD Power", rFPower);
            dashboardTelemetry.addData("AI Power", lBPower);
            dashboardTelemetry.addData("AD Power", rBPower);
            dashboardTelemetry.addData("IMU Heading: ", telemetryImu());
            dashboardTelemetry.update();
        }
    }

    public double telemetryImu() {
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

}
