Author: Mattia Piras
Assignment3 for the Reaearch Track Course of Robotics engineering degree 
---------------------------------------------------------------------------------------------------------
IDEAS:
errore1: non vede le funzioni della classe controller ( forse anche quelle della classe user) come membri di essa.

Il fine dell'assignment è quello di dare la posssibilità all'utente di scegliere tra
 - Autonomous driving
 - Manual driving (usando teleop_keyboard)
 - Assisted drivign (uguale a prima ma nel caso l'utente decidesse di prendere una direzione verso un muro questa nonsarebbe consentita)
 
 Viene data all'utente la possibilità di cancellare il goal tramite l'action client(riferito al action server move_base)
 
 Ho utilizzato:
 - due classi
 - due personal services:
  - Goal_service per spedire il goal una volta inserite le coordinate
  - Behavior_mode_service per spedire il modo scelto dall'utente per guidare il robot
  
 - parameter server
  - due valori 
   - per il timeout
   - per treshold dall'ostacolo
   
 - Pub e Sub:
  - per mandare il messaggio di cancellazione se l'utente preme q
  - scan laser per avoidanceobstacle
  - controller_cmd_vel in pratica è cmd_vel ma rimappata nel launch file in modo da gestire la direzione del robot prima di pubblicarla per assisted driving
  - state info (ABBASTANZA INUTILE CREDO CHE LO LEVERÒ. HO PENSATO DI USARLO NEL CASO IN CUI ACTION.RESULT SIA IN QUALCHE MODO IMPORTANTE PER L'UTENTE E QUESTO VOLESSE VEDERLO STAMPATO A SCHERMO)
 
 - Action Client
  - gestisce il goal da pubblicare e la cancellazione come anche il feedback(utile per sapere il robot è giunto a destinazione entro un tempo prestabilito o meno)
