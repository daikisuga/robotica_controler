/*
  Exemplo introdutório de uso do WeBots
 */


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */

//TIME_STEP é o incremento de tempo usado na simulação
//512 é um valor MUITO alto.... mas ajuda para ler o que está sendo mostrado no console
//para a simulação final, melhor usar valores menores.... 16 ou 32!
#define TIME_STEP 512


#define QtddCaixa 18




/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

int main(int argc, char **argv) {

  
  int i=0;

  

  /* necessary to initialize webots stuff */
   wb_robot_init();
 
   /*
    * You should declare here WbDeviceTag variables for storing
    * robot devices like this:
    *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
    *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
    */
  
  
  
  //exemplo para referenciar um objeto no mundo
  //    Alterar a proprieddade SUPERVISOR do e-Puck para TRUE
  //    Mudar o DEF da Wooden Box para CAIXA
   WbNodeRef caixa[QtddCaixa];

   char nomeCaixa[10]={0};
   
   for(i=0;i<QtddCaixa;i++){
       sprintf(nomeCaixa,"CAIXA%02d",i); //form os nomes dos sensores
       caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
       if(caixa[i]!=NULL)
          printf("%2d. %s  -  %p\n",i,nomeCaixa,(void*)caixa[i]);
       else
          printf("Falha ao carregar a posição da %s\n",nomeCaixa);
    }
    printf("\n\n CAIXAS OK  \n\n");
  
  

  
  

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  while (wb_robot_step(TIME_STEP) != -1) {
    

    //lendo posiçao das caixas
    printf("           X       Y      Z\n");
    for(i=0;i<QtddCaixa;i++){
      const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa[i]);
      printf("CAIXA%02d %5.2f   %5.2f  %5.2f\n\n",i+1,PosicaoCaixa[0],PosicaoCaixa[1],PosicaoCaixa[2]);
        }

  };


  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();

  return 0;

}