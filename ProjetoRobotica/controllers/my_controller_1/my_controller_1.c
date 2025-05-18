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
#define THRESHOLD_MOV 0.005
#define ESPERA_MOVIMENTO 60
#define MAX_COLISOES_ENROSCO 3
#define INTERVALO_ENROSCO_PASSOS 94
#define ENROSCO_DIST_LIMIAR 0.03

int passos_desde_primeira_colisao = -1;
int colisoes_recentes = 0;
double ultima_vl = 0.0, ultima_vr = 0.0;

int main() {
  wb_robot_init();
  srand(time(NULL));

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  WbDeviceTag prox[NUM_SENSORS];
  char sensor_name[5];
  for (int i = 0; i < NUM_SENSORS; i++) {
    sprintf(sensor_name, "ps%d", i);
    prox[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(prox[i], TIME_STEP);
  }

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

  double v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  double v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
  wb_motor_set_velocity(left_motor, v_left);
  wb_motor_set_velocity(right_motor, v_right);

  int timer = 100;
  static double ultima_pos_robo[3] = {0};

  while (wb_robot_step(TIME_STEP) != -1) {
    printf("Timer: %d\n", timer);

    int bateu = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      double val = wb_distance_sensor_get_value(prox[i]);
      if (val > 80.0) bateu = 1;
    }

    if (bateu) {
      printf("COLIDIU!\n");
      for (int t = 0; t < ESPERA_MOVIMENTO; t++) wb_robot_step(TIME_STEP);

      int dancou = 0;
      for (int i = 0; i < NUM_BOXES; i++) {
        if (!caixas[i]) continue;
        const double *nova_pos = wb_supervisor_node_get_position(caixas[i]);
        if (nova_pos && (
              fabs(nova_pos[0] - caixa_pos_ant[i][0]) > THRESHOLD_MOV ||
              fabs(nova_pos[2] - caixa_pos_ant[i][2]) > THRESHOLD_MOV)) {

          printf("Caixa %d se moveu! DANÇA!\n", i + 1);

          while(true) {
            wb_motor_set_velocity(left_motor, MAX_SPEED);
            wb_motor_set_velocity(right_motor, -MAX_SPEED);
            wb_robot_step(TIME_STEP);
          }
        }
      }

      // Lógica de enrosco
      printf("Nenhuma caixa se moveu.\n");
      if (passos_desde_primeira_colisao < 0) {
        passos_desde_primeira_colisao = 0;
        colisoes_recentes = 1;
      } else {
        passos_desde_primeira_colisao++;
        colisoes_recentes++;
      }

      const double *pos_robo = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
      double dx = pos_robo[0] - ultima_pos_robo[0];
      double dz = pos_robo[2] - ultima_pos_robo[2];
      double dist = sqrt(dx * dx + dz * dz);

      if (passos_desde_primeira_colisao == 1) {
        ultima_pos_robo[0] = pos_robo[0];
        ultima_pos_robo[1] = pos_robo[1];
        ultima_pos_robo[2] = pos_robo[2];
      }

      if (passos_desde_primeira_colisao > INTERVALO_ENROSCO_PASSOS) {
        passos_desde_primeira_colisao = -1;
        colisoes_recentes = 0;
      } else if (colisoes_recentes >= MAX_COLISOES_ENROSCO && dist < ENROSCO_DIST_LIMIAR) {
        printf("Modo enrosco ativado!\n");
        int sentido = (rand() % 2 == 0) ? 1 : -1;
        for (int i = 0; i < 20; i++) {
          wb_motor_set_velocity(left_motor, sentido * MAX_SPEED);
          wb_motor_set_velocity(right_motor, -sentido * MAX_SPEED);
          wb_robot_step(TIME_STEP);
        }
        for (int i = 0; i < 30; i++) {
          wb_motor_set_velocity(left_motor, MAX_SPEED);
          wb_motor_set_velocity(right_motor, MAX_SPEED);
          wb_robot_step(TIME_STEP);
        }
        passos_desde_primeira_colisao = -1;
        colisoes_recentes = 0;
      }

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
      timer = 0;
    }

    if (timer-- <= 0) {
      v_left = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      v_right = ((double)(rand() % 100) / 100.0) * MAX_SPEED;
      ultima_vl = v_left;
      ultima_vr = v_right;
      wb_motor_set_velocity(left_motor, v_left);
      wb_motor_set_velocity(right_motor, v_right);
      timer = 100 + rand() % 100;
    }
  }

  wb_robot_cleanup();
  return 0;
}
