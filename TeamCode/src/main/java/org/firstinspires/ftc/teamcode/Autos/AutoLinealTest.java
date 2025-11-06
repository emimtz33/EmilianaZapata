package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto IMU by Raúl", group = "Tests")
public class AutoLinealTest extends LinearOpMode {
    //Variables globales
    DcMotor leftDrive, rightDrive;
    public IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //Creación de motores e IMU
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        //Configuración para que los motores funcionen con encoder
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Inicializar IMU
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        waitForStart();

        while(opModeIsActive()){
            giroAngulo(180);
            sleep(2000);
            avanzarRecto(0.4,10);
            sleep(2000);
            giroAngulo(270);
            sleep(3000);
            avanzarRecto(0.4,10);
            sleep(2000);
            //giroAngulo(0);
            //sleep(3000);
            avanzarRecto(-0.4,10);
            sleep(2000);
            //giroAngulo(90);
            //sleep(3000);
            avanzarRecto(-0.4,10);
            sleep(2000);
            break;
        }
    }

    // Creación de funciones adicionales //

    //Función para giro preciso
    public void giroAngulo(double angulo){
        double kp = 0.015;       // Ganancia proporcional (ajustable)
        double minPower = 0.12;  // Potencia mínima para vencer fricción estática
        double tolerancia = 0.5; // Error aceptable en grados

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
            leftDrive.setPower(power);
            rightDrive.setPower(-power);

            telemetry.addData("Ángulo actual", obtenerAngulo());
            telemetry.addData("Error", error);
            telemetry.addData("Potencia", power);
            telemetry.update();
        }

        // Detener motores al final del giro
        poderMotor(0);
    }

    //Función para obtener ángulo
    public double obtenerAngulo(){
        return -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))+180;
    }

    //Funcionar para calcular error (posición objetivo - posición actual)
    public double obtenerError(double anguloObjetivo){
        //Calcular el error (distancia al ángulo objetivo)
        double error = anguloObjetivo - obtenerAngulo();
        if(error > 180)error -= 360;
        if(error < -180)error += 360;
        return error;
    }

    //Función para avanzar recto
    public void avanzarRecto(double poder, double distanciaCM){
        double targetAngle = obtenerAngulo();
        double kp = 0.015;
        // Calcular ticks necesarios (ajusta según tus motores y ruedas)
        double ticksPorVuelta = 280;
        double diametroRuedaCM = 9.2;
        double ticksPorCM = ticksPorVuelta / (Math.PI * diametroRuedaCM); //28.88
        int ticksObjetivo = (int)(distanciaCM * ticksPorCM);

        // Resetear encoders y configurar modo
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(ticksObjetivo);
        rightDrive.setTargetPosition(ticksObjetivo);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(poder);
        rightDrive.setPower(poder);

        leftDrive.getCurrentPosition();
        leftDrive.getDirection();
        rightDrive.getDirection();

        while (opModeIsActive() &&
                Math.abs(leftDrive.getCurrentPosition()) < ticksObjetivo &&
                Math.abs(rightDrive.getCurrentPosition()) < ticksObjetivo) {

            double error = obtenerError(targetAngle);
            double correction = error * kp;

            // Limitar la corrección para evitar sobresaturación
            correction = Math.max(Math.min(correction, 0.3), -0.3);

            // Aplicar corrección (izquierda o derecha)
            leftDrive.setPower(poder + correction);
            rightDrive.setPower(poder - correction);

            telemetry.addData("Angulo", leftDrive.getCurrentPosition());
            telemetry.addData("DireccionMotorIzquierdo", leftDrive.getDirection());
            telemetry.addData("DireccionMotorDerecho", rightDrive.getDirection());
            telemetry.addData("Error", error);
            telemetry.addData("Corrección", correction);
            telemetry.addData("Heading: ", obtenerAngulo());
            telemetry.addData("L", leftDrive.getCurrentPosition());
            telemetry.addData("R", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        poderMotor(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Funciones de giro
    public void enfrente(double poder){
        leftDrive.setPower(poder);
        rightDrive.setPower(poder);
    }
    public void atras(double poder){
        leftDrive.setPower(-poder);
        rightDrive.setPower(-poder);
    }
    public void derecha(double poder){
        leftDrive.setPower(poder);
        rightDrive.setPower(-poder);
    }
    public void izquierda(double poder){
        leftDrive.setPower(-poder);
        rightDrive.setPower(poder);
    }

    public void poderMotor(double poder){
        leftDrive.setPower(poder);
        rightDrive.setPower(poder);
    }
}