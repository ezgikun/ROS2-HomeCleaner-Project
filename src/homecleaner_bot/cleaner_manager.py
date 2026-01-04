#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


CLEANING_POINTS = [
    [-5.73, 4.57],  # 1. Nokta (Mutfak)
    [-6.62, 0.102],  # 2. Nokta (Salon)
    [-5.45, -4.58],   # 3. Nokta (Koridor)
    [1.09, -4.33],    # 4. Nokta (Oda 1)
    [6.38, -4.49],   # 5. Nokta (Oda 2)
    [6.39, 4.81]     # 6. Nokta (Oda 3)
]
HOME_POSE = [0.0, 0.0]

def create_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    button_node = rclpy.create_node('button_listener')
    
    state = "IDLE" 
    current_index = 0
    stop_signal = False
    first_start = False 
    
    last_click_time = 0
    
    def button_callback(msg):
        nonlocal state, stop_signal, last_click_time, first_start
        
        if (time.time() - last_click_time) < 1.0: return
        last_click_time = time.time()
        
        print(f"\n[BUTON] Tiklandi! (Durum: {state})")
        
        if state == "IDLE":
            print(">>> BASLAT: Temizlik Moduna Geciliyor...")
            state = "CLEANING"
            stop_signal = False
            first_start = True 
        else:
            print(">>> DURDUR: Acil Durum! Eve donuluyor...")
            stop_signal = True
            navigator.cancelTask()
            state = "RETURNING"

    button_node.create_subscription(PointStamped, '/clicked_point', button_callback, 10)
    
    print("SISTEM HAZIR (V6 - AGRESIF BASLANGIC)... Nav2 Bekleniyor...")
    navigator.waitUntilNav2Active()
    print("NAV2 BAGLANDI! RViz 'Publish Point' butonu bekleniyor.")


    while rclpy.ok():
        rclpy.spin_once(button_node, timeout_sec=0.1)
        

        if state == "CLEANING":

            if first_start:
                pt = CLEANING_POINTS[current_index]
                print(f"\n[BASLANGIC] Hedef {current_index+1} Gonderiliyor: {pt}")
                goal = create_pose(navigator, pt[0], pt[1])
                navigator.goToPose(goal)
                first_start = False 
            

            elif not navigator.isTaskComplete():
                pass 
            

            else:
                result = navigator.getResult()
                

                if current_index < len(CLEANING_POINTS):

                    current_index += 1
                    if current_index < len(CLEANING_POINTS):
                        pt = CLEANING_POINTS[current_index]
                        print(f"\n[DEVAM] Hedef {current_index+1} Gonderiliyor: {pt}")
                        goal = create_pose(navigator, pt[0], pt[1])
                        navigator.goToPose(goal)
                    else:
                        print("\n[BILGI] Liste Bitti! Eve donuluyor.")
                        state = "RETURNING"
                        current_index = 0
                else:
                    state = "RETURNING"


        elif state == "RETURNING":
            if navigator.isTaskComplete():
                 print(f"[EVE DONUS] Rota: {HOME_POSE}")
                 home_goal = create_pose(navigator, HOME_POSE[0], HOME_POSE[1])
                 navigator.goToPose(home_goal)
                 state = "MOVING_HOME"


        elif state == "MOVING_HOME":
            if navigator.isTaskComplete():
                print("[SON] Robot Park Etti. Bosta.")
                state = "IDLE"
                current_index = 0
                first_start = False

    navigator.lifecycleShutdown()
    button_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
