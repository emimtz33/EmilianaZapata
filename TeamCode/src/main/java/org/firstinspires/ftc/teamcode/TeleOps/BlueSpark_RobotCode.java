package org.firstinspires.ftc.teamcode.Projectos;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BlueSpark RobotCode")
public class BlueSpark_RobotCode extends LinearOpMode {

    public IMU imu;

    @Override
    public void runOpMode(){

        telemetry.addData("Estado", "Inicializado correctamente");
        telemetry.update();

        // Inicializar motores
        DcMotorEx leftFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftFront");
        DcMotorEx rightFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightFront");
        DcMotorEx leftBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftBack");
        DcMotorEx rightBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightBack");

        //IMU object creation
        imu = hardwareMap.get(IMU.class, "imu");

        //Initialize the hardware variables
        DcMotorEx Shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter");
        DcMotor Intake = hardwareMap.get(DcMotor.class, "intake1");
        DcMotor Regulator = hardwareMap.get(DcMotor.class, "Regulator");

        // Configure robot orientation
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // Function orientations
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Regulator.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Servo object creation


        // IMU object creation
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
        );



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

            //Intake, shooter and regulator variables
            double shooterPower;
            double intakePower;
            double regulatorPower;

            // Shooter "if" statements
            if(gamepad1.right_trigger < 0.1){
                shooterPower = 2500;
            }else shooterPower = 0;

            if(gamepad1.right_bumper){
                shooterPower = 1700;
            }

            // Intake "if" statements
            if(gamepad1.left_trigger < 0.1){
                intakePower = 0.8;
            }else intakePower = 0;

            if (gamepad1.left_bumper){
                intakePower = -0.7;
            }

            // Regulator function conditions
            if(Shooter.getVelocity() < 2530 && Shooter.getVelocity() > 2470){
                regulatorPower = 0.8;
            }else regulatorPower = 0;

            if (Shooter.getVelocity() < 1730 && Shooter.getVelocity() > 1670){
                regulatorPower = 0.8;
            }


            //Give power to Driver motors
            leftFront.setVelocity(lFPower * 1500);
            rightFront.setVelocity(rFPower * 1500);
            leftBack.setVelocity(lBPower * 1500);
            rightBack.setVelocity(rBPower * 1500);

            //Give power to Operator motors
            Shooter.setVelocity(shooterPower);
            Intake.setPower(intakePower);
            Regulator.setPower(regulatorPower);

            // Punishment sistem to avoid hurting Gracious Professionalism
            if(gamepad2.dpad_left){
                gamepad1.rumbleBlips(1);
            }

            if(gamepad1.dpad_left){
                gamepad2.rumbleBlips(1);
            }


            //Telemetry
            telemetry.addLine("Operator Telemetry");
            telemetry.addData("Poder Intake: ", Intake.getPower());
            telemetry.addData("Regulator Power", Regulator.getPower());
            telemetry.addData("Velocidad Shooter", Shooter.getVelocity());
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

        }
    }

    public double telemetryImu() {
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

}

