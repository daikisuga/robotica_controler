#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define NUM_SENSORS 8
#define MAX_SPEED 6.28
#define NUM_BOXES 20
#define THRESHOLD_MOV 0.005  // tolerância para detectar movimento da caixa
#define ESPERA_MOVIMENTO 60  // passos de simulação (~2s)

int main() {
  wb_robot_init();
  srand(time(NULL));

  // Motores
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  // Sensores de proximidade
  WbDeviceTag prox[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ps%d", i);
    prox[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(prox[i], TIME_STEP);
  }

  // Referência para caixas
  WbNodeRef caixas[NUM_BOXES];
  double caixa_pos_ant[NUM_BOXES][3];
  char def[16];
  for (int i = 0; i < NUM_BOXES; i++) {
    sprintf(def, "CAIXA%d", i + 1);
    caixas[i] = wb_supervisor_node_get_from_def(def);
    const double *pos = (caixas[i]) ? wb_supervisor_node_get_position(caixas[i]) : NULL;
    if (pos) {
      caixa_pos_ant[i][0] = pos[0];
      caixa_pos_ant[i][1] = pos[1];
      caixa_pos_ant[i][2] = pos[2];
    }
  }

  // Movimento inicial aleatório
  double v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  double v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  wb_motor_set_velocity(left_motor, v_left);
  wb_motor_set_velocity(right_motor, v_right);

  int timer = 100;

  while (wb_robot_step(TIME_STEP) != -1) {
    printf("Timer: %d\n", timer);

    // Verifica colisão
    int bateu = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      double val = wb_distance_sensor_get_value(prox[i]);
      if (val > 80.0) bateu = 1;
    }

    if (bateu) {
      printf("COLIDIU!\n");

      // Espera antes de verificar movimento
      for (int t = 0; t < ESPERA_MOVIMENTO; t++) wb_robot_step(TIME_STEP);

      // Verifica se alguma caixa se moveu
      int dancou = 0;
      for (int i = 0; i < NUM_BOXES; i++) {
        if (!caixas[i]) continue;

        const double *nova_pos = wb_supervisor_node_get_position(caixas[i]);
        if (nova_pos && (
              fabs(nova_pos[0] - caixa_pos_ant[i][0]) > THRESHOLD_MOV ||
              fabs(nova_pos[2] - caixa_pos_ant[i][2]) > THRESHOLD_MOV)) {

          printf("Caixa %d se moveu! DANÇA!\n", i + 1);

          // Dança: gira no próprio eixo
          while(true) {
            wb_motor_set_velocity(left_motor, MAX_SPEED);
            wb_motor_set_velocity(right_motor, -MAX_SPEED);
            wb_robot_step(TIME_STEP);
          }

          dancou = 1;
          break;
        }
      }

      if (!dancou) {
        printf("Nenhuma caixa se moveu.\n");
      }

      // Atualiza posições antigas
      for (int i = 0; i < NUM_BOXES; i++) {
        if (caixas[i]) {
          const double *pos = wb_supervisor_node_get_position(caixas[i]);
          if (pos) {
            caixa_pos_ant[i][0] = pos[0];
            caixa_pos_ant[i][1] = pos[1];
            caixa_pos_ant[i][2] = pos[2];
          }
        }
      }

      // Reinicia timer e movimento aleatório
      timer = 0;
    }

    // Movimento aleatório a cada N passos
    if (timer-- <= 0) {
      v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      wb_motor_set_velocity(left_motor, v_left);
      wb_motor_set_velocity(right_motor, v_right);
      timer = 100 + rand() % 100;
    }
  }

  wb_robot_cleanup();
  return 0;
}
