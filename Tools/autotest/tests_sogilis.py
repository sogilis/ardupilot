import arducopter
import time

def arm_disarm (mavproxy, mav):
    '''A scripted flight plan'''
    if (arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.disarm_motors(mavproxy,mav)):
        return True
    return False

def failsafe(mavproxy, mav):
    '''A scripted flight plan'''
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=80, takeoff_throttle=1510)  and
        arducopter.hover(mavproxy,mav, hover_throttle=1300) and
        arducopter.fly_throttle_failsafe(mavproxy, mav, side=80, timeout=120)
    ):
        return True
    return False

def takeoff (mavproxy, mav):
    '''A scripted flight plan'''
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=30, takeoff_throttle=1510)):
        return True
    return False

def guided_test (mavproxy, mav):
    '''A scripted flight plan'''
    wps = [
        [45.22298, 5.690054], 
        [45.223,   5.690031],
        [45.22308, 5.689955],
        [45.22316, 5.689875],
        [45.22324, 5.689797],
        [45.22331, 5.68972],
        [45.22338, 5.68964],
        [45.22344, 5.689583],
        [45.22351, 5.689513],
        [45.22359, 5.689423],
        [45.22365, 5.689343],
        [45.22372, 5.689287],
        [45.22379, 5.689219],
        [45.22383, 5.689182],
        [45.22383, 5.689187],
        [45.22383, 5.689188],
        [45.22382, 5.6892],
        [45.22378, 5.689222],
        [45.22371, 5.689301],
        [45.22364, 5.689387],
        [45.22356, 5.689456],
        [45.2235,  5.68953],
        [45.22344, 5.6896],
        [45.22337, 5.689683],
        [45.22332, 5.689734],
        [45.22324, 5.689822],
        [45.22317, 5.689898],
        [45.22311, 5.68997],
        [45.22306, 5.690037],
        [45.22299, 5.690111],
        [45.2229,  5.690189],
        [45.22282, 5.69026],
        [45.22275, 5.690337],
        [45.22268, 5.690403],
        [45.22261, 5.690464],
        [45.22259, 5.690482],
        [45.2226,  5.690475],
        [45.2226,  5.690466],
        [45.2226,  5.690466],
        [45.22262, 5.69045],
        [45.22268, 5.690374],
        [45.22276, 5.690294],
        [45.22284, 5.690213],
        [45.2229,  5.690146],
        [45.22293, 5.690103],
        ]
 

    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=10, takeoff_throttle=1510) and
        arducopter.set_guided_mode(mavproxy,mav)):
        
            #arducopter.param_set (mavproxy, mav, "WPNAV_SPEED" , 700)
            #i = 0 ; 
            for wp in wps:
                arducopter.goto_guided_point(mavproxy, mav, lat=wp[0], lng=wp[1], alt=10, freq=2) 
                #i += 1 
                #if (i >= 20):
                    #arducopter.param_set (mavproxy, mav, "WPNAV_SPEED" , 200)
            return True
        
        #arducopter.goto_guided_point(mavproxy, mav, lat=wp1[0], lng=wp1[1], alt=20, freq=30) and
        #arducopter.goto_guided_point(mavproxy, mav, lat=wp2[0], lng=wp2[1], alt=20, freq=5) and
        #arducopter.goto_guided_point(mavproxy, mav, lat=wp3[0], lng=wp3[1], alt=20, freq=5) and
        #arducopter.goto_guided_point(mavproxy, mav, lat=wp4[0], lng=wp4[1], alt=20, freq=30)):
        #arducopter.goto_guided_point(mavproxy, mav, lat=45.222711, lng=5.690787, alt=20, freq=10) and
        #arducopter.goto_guided_point(mavproxy, mav, lat=45.27, lng=5.68, alt=20, freq=10)):
   
    return False


def change_speed (mavproxy, mav):
    result = True
    speed = 7.0
    max_speed = speed + 0.5
    min_speed = speed - 1.0
    '''Change speed'''
    mavproxy.send("param set WPNAV_ACCEL 150\n")
    mavproxy.send("param set WP_YAW_BEHAVIOR 0\n")
    
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=10, takeoff_throttle=1510) and
        arducopter.set_guided_mode(mavproxy,mav)):
            
            mavproxy.send('guided ' + str(45.2262611111) + ' ' + str(5.6936361111) + ' ' + str(10) + '\n')
            mavproxy.send('speed ' + str(speed) + '\n')
            
            tstart = time.time()
            print("\nWait 10s for Acceleration")
            while time.time() < tstart + 10:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
            
            tstart = time.time()
            print("\nCheck speed between " + str(min_speed) + " and " + str(max_speed) + " during 30s\n")
            while time.time() < tstart + 30:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.airspeed < max_speed) and (vfr_hud_msg.airspeed > min_speed)
                print("Speed: " + str(vfr_hud_msg.airspeed))
                
    return result


def change_speed2 (mavproxy, mav):
    result = True
    speed1 = 6.0
    speed2 = 2.5
    max_offset = 0.5
    min_offset = 1.0
    '''Change speed with 2 thresholds'''
    mavproxy.send("param set WPNAV_ACCEL 150\n")
    mavproxy.send("param set WP_YAW_BEHAVIOR 0\n")
    
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=10, takeoff_throttle=1510) and
        arducopter.set_guided_mode(mavproxy,mav)):

            
            mavproxy.send('guided ' + str(45.2274694444) + ' ' + str(5.686075) + ' ' + str(10) + '\n')

            mavproxy.send('speed ' + str(speed1) + '\n')
            tstart = time.time()
            print("\nWait 10s for acceleration")
            while time.time() < tstart + 10:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                print("Speed: " + str(vfr_hud_msg.airspeed))
            
            min_speed = speed1 - min_offset
            max_speed = speed1 + max_offset
            tstart = time.time()
            print("\nCheck speed between " + str(min_speed) + " and " + str(max_speed) + " during 20s\n")
            while time.time() < tstart + 20:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.airspeed < max_speed) and (vfr_hud_msg.airspeed > min_speed)
                print("Speed: " + str(vfr_hud_msg.airspeed))
                
            mavproxy.send('speed ' + str(speed2) + '\n')
            tstart = time.time()
            print("\nWait 5s for deceleration")
            while time.time() < tstart + 5:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                print("Speed: " + str(vfr_hud_msg.airspeed))
            
            min_speed = speed2 - min_offset
            max_speed = speed2 + max_offset
            tstart = time.time()
            print("\nCheck speed between " + str(min_speed) + " and " + str(max_speed) + " during 20s\n")
            while time.time() < tstart + 20:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.airspeed < max_speed) and (vfr_hud_msg.airspeed > min_speed)
                print("Speed: " + str(vfr_hud_msg.airspeed))
                
    return result


def change_yaw (mavproxy, mav):
    result = True
    angle = 180
    angle_min = 178
    angle_max = 182
    mavproxy.send("param set WP_YAW_BEHAVIOR 0\n")
    
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=10, takeoff_throttle=1510) and
        arducopter.set_guided_mode(mavproxy,mav)):
            
            mavproxy.send('guided ' + str(45.2227) + ' ' + str(5.6962055556) + ' ' + str(10) + '\n')
            mavproxy.send('yaw ' + str(angle)  + ' ' + str(45.0) + ' ' + str(0) + '\n')
            
            tstart = time.time()
            print("\nWait 10s for rotation")
            while time.time() < tstart + 10:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                print("Heading: " + str(vfr_hud_msg.heading))
            
            tstart = time.time()
            print("\nCheck heading between " + str(angle_min) + " and " + str(angle_max) + " during 30s\n")
            while time.time() < tstart + 30:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.heading < angle_max) and (vfr_hud_msg.heading > angle_min)
                print("Heading: " + str(vfr_hud_msg.heading))
                
    return result

    
def change_yaw2 (mavproxy, mav):
    result = True
    angle_1 = 45
    angle_2 = 225
    mavproxy.send("param set WP_YAW_BEHAVIOR 0\n")
    
    if ( 
        arducopter.calibrate_level(mavproxy, mav) and
        arducopter.arm_motors(mavproxy, mav) and  
        arducopter.takeoff(mavproxy,mav, alt_min=10, takeoff_throttle=1510) and
        arducopter.set_guided_mode(mavproxy,mav)):
            
            mavproxy.send('guided ' + str(45.2179944444) + ' ' + str(5.6897027778) + ' ' + str(10) + '\n')
            mavproxy.send('yaw ' + str(angle_1)  + ' ' + str(45.0) + ' ' + str(0) + '\n')
            
            tstart = time.time()
            print("\nWait 10s for rotation heading = 45")
            while time.time() < tstart + 10:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                print("Heading: " + str(vfr_hud_msg.heading))
            
            tstart = time.time()
            print("\nCheck heading between " + str(43) + " and " + str(47) + " during 15s\n")
            while time.time() < tstart + 15:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.heading > 43) and (vfr_hud_msg.heading < 47)
                print("Heading: " + str(vfr_hud_msg.heading))
                
            mavproxy.send('yaw ' + str(angle_2)  + ' ' + str(45.0) + ' ' + str(0) + '\n')
            tstart = time.time()
            print("\nWait 10s for rotation heading = 225")
            while time.time() < tstart + 10:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                print("Heading: " + str(vfr_hud_msg.heading))
            
            tstart = time.time()
            print("\nCheck heading between " + str(223) + " and " + str(227) + " during 15s\n")
            while time.time() < tstart + 15:
                vfr_hud_msg  = mav.recv_match(type='VFR_HUD' , blocking=True)
                result = result and (vfr_hud_msg.heading > 223) and (vfr_hud_msg.heading < 227)
                print("Heading: " + str(vfr_hud_msg.heading))
                
    return result


