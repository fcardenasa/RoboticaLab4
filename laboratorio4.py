import os
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState
import rospy
import numpy as np
from cmath import pi

_author_ = "Fernando Cardenas Acosta, Raul Ignacion Marin Medina"
_email_ = "fcardenasa@unal.edu.co, ramarinm@unal.edu.co"
_status_ = "Test"



ID_arts = [1,2,3,4,5] #quitar

gradosPosiciones = [0,0,0,0,0]

puntos = {0:(0,0,0,0,0),
          1:(25,25,20,-20,0),
          2:(-35,35,-30,30,0),
          3:(85,-20,55,25,0),
          4:(80,-35,55,-45,0)}

torques = (800,600,600,500,500)

proxPos = puntos[0]

tiempoEjecucion = 1  

def callback(data):
    global gradosPosiciones
    gradosPosiciones = np.round(np.multiply(data.posicion, 180/pi),3)
    
def impError():
    print ("           Cadera | Hombro | Codo | Muneca | Garra")
    print ("Posicion: ", end= " ")
    
    for i in range(5):
          print(f"{gradosPosiciones[i]} º", end= " ")
          
    print()
    print ("Error:    ", end = " ")
    
    for i in range(5):
          print(f"{np.round(proxPos[i] - gradosPosiciones[i],3)} º", end= " ")
          
    print()

def conversorAng(grados):
    datos = round((1024/300)*grados + 512)
    if datos > 1023: 
        print('Verificar los datos de configuracion de articulacion')
        return 1023
    elif datos < 0: 
        print('Verificar los datos de configuracion de articulacion')
        return 0
    else: 
        return datos

def movimiento(posicion):
    global proxPos
    proxPos = puntos[posicion]
    
    if posicion == 0:
        posicion = "home"
        
    print(f"Siguiente posicion {posicion}: {proxPos} \n")
    
    for i in range(5):
        print(f"Moviendo articulacion: {i+1} \n\n")
        art(i+1, conversorAng(proxPos[i]))

def art(id, posicion):
    comandoArt('', id, 'Goal_Position', posicion, tiempoEjecucion)

def comandoArt(comando, numID, nomDir, valor, tiempoPausa):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(comando,numID,nomDir,valor)
        rospy.sleep(tiempoPausa)
        return result.comm_result
    except rospy.ServiceException as exc:   
        print(str(exc))

def initial_config():
    for i in range(5):
        comandoArt('', i+1, 'Torque_Limit', torques[i], 0)
    for i in range(5):
        art(i+1, conversorAng(0))
    return

def listener():
    rospy.init_node('joint_listener', anonymous = True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)

def main():
    os.system('clear')
    try: 
        listener()
        initial_config()
        while True:
            print("Bienvenido al sistema de control del robot Phantom disenado por:")
            print("Raul Ignacio Marin Medina")
            print("Fernando Cardenas Acosta.")
            print("")
            print("Elija una opcion de posicion:")
            print("Posicion Home:   0")
            print("Posicion 1:      1")
            print("Posicion 2:      2")
            print("Posicion 3:      3")
            print("Posicion 4:      4\n")
            print("Exit Program:    E\n")
            

            carEleccion = str(input("Seleccione la posicion que desee: "))
            
            if (carEleccion == "e" or carEleccion =="E"):
                for i in range(5):
                    art(i+1, conversorAng(0))
                break
                
            
            try:
                
                
                numEleccion = int(carEleccion)
                
                if (numEleccion > 4  or numEleccion < 0):
                    print("Seleccione una posicion valida")
                    input()
                else:
                   
                    movimiento(numEleccion)
                    impError()
                    input()     
            except:
                print("Debe escribir un numero")
                input()
            os.system('clear')
    except:
        print('No se pudo conectar con el robot')
        pass

# Main function
if __name__ == '__main__':
    main()
