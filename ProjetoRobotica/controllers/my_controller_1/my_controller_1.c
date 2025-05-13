#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256
#define QtddCaixas 20

int main() {
  int i;
  char texto[TamanhoTexto] = {0};
  double LeituraSensorProx[QtddSensoresProx];

  wb_robot_init();

  // Referência ao próprio robô
  WbNodeRef robo = wb_supervisor_node_get_self();

  // Motores
  WbDeviceTag MotorEsquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireito = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  // Sensores de proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];
  char nomeSensor[10];
  for(i = 0; i < QtddSensoresProx; i++) {
    sprintf(nomeSensor, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(nomeSensor);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  // LED
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], 1);

  // Carrega referências às caixas
  WbNodeRef caixas[QtddCaixas];
  char nomeCaixa[16];
  for (i = 0; i < QtddCaixas; i++) {
    sprintf(nomeCaixa, "CAIXA%d", i + 1);
    caixas[i] = wb_supervisor_node_get_from_def(nomeCaixa);
  }

  // Loop principal
  while (wb_robot_step(TIME_STEP) != -1) {
    // Pisca o LED
    wb_led_set(Leds[0], !wb_led_get(Leds[0]));

    // Lê posição do robô
    const double *posRobo = wb_supervisor_node_get_position(robo);

    // Encontra a caixa mais próxima
    const double *posMaisPerto = NULL;
    double menorDistancia = 1e9;
    for (i = 0; i < QtddCaixas; i++) {
      const double *posCaixa = wb_supervisor_node_get_position(caixas[i]);
      double dx = posCaixa[0] - posRobo[0];
      double dz = posCaixa[2] - posRobo[2];
      double dist = sqrt(dx * dx + dz * dz);
      if (dist < menorDistancia) {
        menorDistancia = dist;
        posMaisPerto = posCaixa;
      }
    }

    // Calcula direção para a caixa
    double dx = posMaisPerto[0] - posRobo[0];
    double dz = posMaisPerto[2] - posRobo[2];
    double angulo = atan2(dz, dx);

    // Calcula direção atual do robô (aproximado pelo eixo Z)
    const double *orientacao = wb_supervisor_node_get_orientation(robo);
    double dirRoboX = orientacao[0];
    double dirRoboZ = orientacao[2];
    double anguloRobo = atan2(dirRoboZ, dirRoboX);

    double erro = angulo - anguloRobo;

    // Normaliza o ângulo entre -PI e PI
    while (erro > M_PI) erro -= 2 * M_PI;
    while (erro < -M_PI) erro += 2 * M_PI;

    double vEsq = 3.0;
    double vDir = 3.0;

    // Ajuste para girar em direção à caixa
    if (fabs(erro) > 0.2) {
      vEsq = -erro * 5;
      vDir = erro * 5;
    }

    // Evita valores fora do limite
    if (vEsq > 6.28) vEsq = 6.28;
    if (vEsq < -6.28) vEsq = -6.28;
    if (vDir > 6.28) vDir = 6.28;
    if (vDir < -6.28) vDir = -6.28;

    wb_motor_set_velocity(MotorEsquerdo, vEsq);
    wb_motor_set_velocity(MotorDireito, vDir);

    // Debug
    sprintf(texto, "Caixa alvo: x=%.2f z=%.2f | Robo: x=%.2f z=%.2f | erro=%.2f",
            posMaisPerto[0], posMaisPerto[2], posRobo[0], posRobo[2], erro);
    printf("%s\n", texto);
  }

  wb_robot_cleanup();
  return 0;
}
