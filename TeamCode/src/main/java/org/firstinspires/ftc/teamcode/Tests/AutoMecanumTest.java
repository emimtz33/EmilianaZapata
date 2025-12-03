package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto Mecanum by Emiliano", group = "Tests")
public class AutoMecanumTest extends LinearOpMode {



    //Variables globales
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotor Intake;

    DcMotorEx shooterR, shooterL;

    CRServo servoIntake;

    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //Creación de motores e IMU
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Break or coast for motors
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");

        //Motor variable creation
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        Intake = hardwareMap.get(DcMotor.class, "intake1");

        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Motor direction configuration
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servo object creation
        CRServo servoIntake;
        servoIntake = hardwareMap.get(CRServo.class, "seervo");

        // Servo direction configuration
        //servoIntake.setDirection(CRServo.Direction.REVERSE);


        // Power variables
        double intakeServoPower = 0;
        double chooserServoPowerR = 0;
        double chooserServoPowerL = 0;
        double shooterPower = 0;
        double regulatorServoPower = 0;

        //Configuración para que los motores funcionen con encoder
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Configure encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
        );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Heading1", obtenerAngulo());
            telemetry.addData("FI Velocidad",leftFront.getVelocity());
            telemetry.addData("FD Velocidad", rightFront.getVelocity());
            telemetry.addData("AI Velocidad", leftBack.getVelocity());
            telemetry.addData("AD Velocidad", rightBack.getVelocity());

            telemetry.addData("FI Posicion",leftFront.getCurrentPosition());
            telemetry.addData("FD Posicion", rightFront.getCurrentPosition());
            telemetry.addData("AI Posicion", leftBack.getCurrentPosition());
            telemetry.addData("AD Posicion", rightBack.getCurrentPosition());
            telemetry.update();
            esperarAuto(1);
            enfrente(500);
            esperarAuto(2);
            derecha(0.3);
            esperarAuto(1);
            izquierda(0.3);
            esperarAuto(1);
            break;
        }
    }


    // Creación de funciones adicionales //


    public void esperarAuto(double seconds) {
        double currentTime = getRuntime();
        while (getRuntime() < currentTime + seconds) {
            sleep(0);
            telemetry.addData("HEADING", obtenerAngulo());
            telemetry.update();
        }
    }

    //Función para giro preciso
    public void giroAngulo(double angulo) {
        double kp = 0.008;       // Ganancia proporcional (ajustable)
        double minPower = 0.07;  // Potencia mínima para vencer fricción estática
        double tolerancia = 0.3; // Error aceptable en grados

        while (opModeIsActive()) {
            double error = obtenerError(angulo);

            // Si el error es suficientemente pequeño, terminamos el giro
            if (Math.abs(error) <= tolerancia) break;

            // Calcular potencia proporcional
            double power = kp * error;

            // Asegurarse de que la potencia no sea demasiado baja
            if (Math.abs(power) < minPower) {
                power = minPower * Math.signum(error);
            }

            // Aplicar giro
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);


            telemetry.addData("Ángulo actual", obtenerAngulo());
            telemetry.addData("Error", error);
            telemetry.addData("Potencia", power);
            telemetry.update();
        }

        // Detener motores al final del giro
        poderMotor(0);
    }



    //Función para obtener ángulo
    public double obtenerAngulo() {
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }


    //Funcionar para calcular error (posición objetivo - posición actual)
    public double obtenerError(double anguloObjetivo) {
        //Calcular el error (distancia al ángulo objetivo)
        double error = anguloObjetivo - obtenerAngulo();
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    //Función para avanzar recto
    public void avanzarRecto(double poder, double distanciaCM) {
        double targetAngle = obtenerAngulo();
        double kp = 0.015;
        // Calcular ticks necesarios (ajusta según tus motores y ruedas)
        double ticksPorVuelta = 280;
        double diametroRuedaCM = 9.2;
        double ticksPorCM = ticksPorVuelta / (Math.PI * diametroRuedaCM); //28.88
        int ticksObjetivo = (int) (distanciaCM * ticksPorCM);

        // Resetear encoders y configurar modo
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(ticksObjetivo);
        leftBack.setTargetPosition(ticksObjetivo);
        rightFront.setTargetPosition(ticksObjetivo);
        rightBack.setTargetPosition(ticksObjetivo);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(poder);
        leftBack.setPower(poder);
        rightFront.setPower(poder);
        rightBack.setPower(poder);

        leftFront.getCurrentPosition();
        leftFront.getDirection();
        rightFront.getDirection();

        while (opModeIsActive() &&
                (leftFront.isBusy() ||  rightFront.isBusy() ||
                        leftBack.isBusy()  ||  rightBack.isBusy())) {

            double error = obtenerError(targetAngle);
            double correction = error * kp;

            // Limitar la corrección para evitar sobresaturación
            correction = Math.max(Math.min(correction, 0.3), -0.3);

            // Aplicar corrección (izquierda o derecha)
            leftFront.setPower(poder + correction);
            leftBack.setPower(poder + correction);
            rightFront.setPower(poder - correction);
            rightBack.setPower(poder - correction);

            telemetry.addData("Angulo", leftFront.getCurrentPosition());
            telemetry.addData("DireccionMotorIzquierdo", leftFront.getDirection());
            telemetry.addData("DireccionMotorDerecho", rightFront.getDirection());
            telemetry.addData("Error", error);
            telemetry.addData("Corrección", correction);
            telemetry.addData("Heading: ", obtenerAngulo());
            telemetry.addData("LeftFront", leftFront.getCurrentPosition());
            telemetry.addData("LeftBack", leftBack.getCurrentPosition());
            telemetry.addData("R", rightFront.getCurrentPosition());
            telemetry.update();
        }

        poderMotor(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void retrocederRecto(double poder, double distanciaCM) {
        double targetAngle = obtenerAngulo();
        double kp = 0.015;
        // Calcular ticks necesarios (ajusta según tus motores y ruedas)
        double ticksPorVuelta = 280;
        double diametroRuedaCM = 9.2;
        double ticksPorCM = ticksPorVuelta / (Math.PI * diametroRuedaCM); //28.88
        int ticksObjetivo = (int) (distanciaCM * ticksPorCM);

        // Resetear encoders y configurar modo
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(ticksObjetivo);
        leftBack.setTargetPosition(ticksObjetivo);
        rightFront.setTargetPosition(ticksObjetivo);
        rightBack.setTargetPosition(ticksObjetivo);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-poder);
        leftBack.setPower(-poder);
        rightFront.setPower(-poder);
        rightBack.setPower(-poder);

        leftFront.getCurrentPosition();
        leftFront.getDirection();
        rightFront.getDirection();

        while (opModeIsActive() &&
                (leftFront.isBusy() ||  rightFront.isBusy() ||
                        leftBack.isBusy()  ||  rightBack.isBusy())) {

            double error = obtenerError(targetAngle);
            double correction = error * kp;

            // Limitar la corrección para evitar sobresaturación
            correction = Math.max(Math.min(correction, 0.3), -0.3);

            // Aplicar corrección (izquierda o derecha)
            leftFront.setPower(poder + correction);
            leftBack.setPower(poder + correction);
            rightFront.setPower(poder - correction);
            rightBack.setPower(poder - correction);

            telemetry.addData("Angulo", leftFront.getCurrentPosition());
            telemetry.addData("DireccionMotorIzquierdo", leftFront.getDirection());
            telemetry.addData("DireccionMotorDerecho", rightFront.getDirection());
            telemetry.addData("Error", error);
            telemetry.addData("Corrección", correction);
            telemetry.addData("Heading: ", obtenerAngulo());
            telemetry.addData("L", leftFront.getCurrentPosition());
            telemetry.addData("R", rightFront.getCurrentPosition());
            telemetry.update();
        }

        poderMotor(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Funcion de intake
    /*public void intake(double poderIntake) {
        intakeServo1.setPower(-(poderIntake));
    }
*/

    /*public void shooter(double shooterVelocity,double chooserServoPower) {
        double regulatorServoPower = 0;

        shooterR.setVelocity(shooterVelocity);
        shooterL.setVelocity(shooterVelocity);
        chooserServoL.setPower(chooserServoPower);
        chooserServoR.setPower(-chooserServoPower);

        double shooterVelocityR = shooterR.getVelocity();
        double shooterVelocityL = shooterL.getVelocity();
        double soltureR = shooterR.getVelocity();
        double soltureL = shooterL.getVelocity();
        double diference = soltureL - soltureR;

        regulationServo.setPower(regulatorServoPower);

        if(shooterVelocityL > 1350 && shooterVelocityR > 1350 && diference < 10 && diference > -10){
            regulatorServoPower = 0.8;
        }

        //Falta telemetria
        telemetry.addData("Poder llanta Negra", regulatorServoPower);
        telemetry.addData("Poder shooters", shooterVelocity);
        telemetry.addData("Poder Chooser Servo", chooserServoPower);
        telemetry.addData("Velocidad shooter derecho", shooterVelocityR);
        telemetry.addData("Velocidad shooter izquierdo", shooterVelocityL);
        telemetry.update();
    }
    */


    //Funciones de giro
    public void enfrente(double velocidad){
        double minPower = 100;
        double maxPower = 2500;

        velocidad = Range.clip((velocidad),minPower, maxPower);


        leftFront.setVelocity(velocidad);
        rightFront.setVelocity(velocidad);
        leftBack.setVelocity(velocidad);
        rightBack.setVelocity(velocidad);

        telemetry.addData("FI Velocidad",leftFront.getVelocity());
        telemetry.addData("FD Velocidad", rightFront.getVelocity());
        telemetry.addData("AI Velocidad", leftBack.getVelocity());
        telemetry.addData("AD Velocidad", rightBack.getVelocity());
        telemetry.update();
    }
    public void atras(double poder){
        leftFront.setPower(-poder);
        rightFront.setPower(-poder);
        leftBack.setPower(-poder);
        rightBack.setPower(-poder);
    }
    public void derecha(double poder){
        leftFront.setPower(poder);
        rightFront.setPower(-poder);
        leftBack.setPower(poder);
        rightBack.setPower(-poder);
    }
    public void izquierda(double poder){
        leftFront.setPower(-poder);
        rightFront.setPower(poder);
        leftBack.setPower(-poder);
        rightBack.setPower(poder);
    }

    public void poderMotor(double poder) {
        leftFront.setPower(poder);
        leftBack.setPower(poder);
        rightFront.setPower(poder);
        rightBack.setPower(poder);
    }
}
