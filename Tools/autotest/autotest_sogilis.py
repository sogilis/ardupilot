
import pexpect, os, sys, shutil, atexit
import fnmatch, time, glob, traceback, signal
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pysim'))
from common import *
from pymavlink import mavutil, mavwp
import random

# get location of scripts
testdir=os.path.dirname(os.path.realpath(__file__))

import util
import arducopter
import tests_sogilis

FRAME='+'
TARGET='sitl'
#HOME=mavutil.location(45.222779,5.690749,10,270)
HOME=mavutil.location(45.2227,5.6897027778,10,270)
AVCHOME=mavutil.location(45.223,5.691,10,0)

tests = [
    'arm_disarm',
    'failsafe',
    'takeoff',
    'guided_test',
    'change_speed',
    'change_speed2',
    'change_yaw',
    'change_yaw2',
    ]


def display (test, res):
    if res:
	 print("TEST : %s PASSED" % test)
    else: 
	 print("TEST : %s FAILED" % test)

print("Running Sogilis Test")

global homeloc

if TARGET != 'sitl':
   util.build_SIL('ArduCopter', target='sitl')

sim_cmd = util.reltopdir('Tools/autotest/pysim/sim_multicopter.py') + ' --frame=%s --rate=400 --home=%f,%f,%u,%u' % (
   FRAME, HOME.lat, HOME.lng, HOME.alt, HOME.heading)
#sim_cmd += ' --wind=6,45,.3'

sil = util.start_SIL('ArduCopter', wipe=True)
mavproxy = util.start_MAVProxy_SIL('ArduCopter', options='--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter')
mavproxy.expect('Received [0-9]+ parameters')

# setup test parameters
mavproxy.send("param load %s/copter_params.parm\n" % testdir)
mavproxy.expect('Loaded [0-9]+ parameters')

# reboot with new parameters
util.pexpect_close(mavproxy)
util.pexpect_close(sil)

sil = util.start_SIL('ArduCopter', height=HOME.alt)
sim = pexpect.spawn(sim_cmd, logfile=sys.stdout, timeout=10)
sim.delaybeforesend = 0
util.pexpect_autoclose(sim)
options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --quadcopter --streamrate=5'

if map:
	options += ' --map'
mavproxy = util.start_MAVProxy_SIL('ArduCopter', options=options)
mavproxy.expect('Logging to (\S+)')
logfile = mavproxy.match.group(1)
print("LOGFILE %s" % logfile)

buildlog = util.reltopdir("../buildlogs/ArduCopter-test.tlog")
print("buildlog=%s" % buildlog)
if os.path.exists(buildlog):
	os.unlink(buildlog)
try:
	os.link(logfile, buildlog)
except Exception:
	pass

# the received parameters can come before or after the ready to fly message
mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])
mavproxy.expect(['Received [0-9]+ parameters', 'Ready to FLY'])

util.expect_setup_callback(mavproxy, expect_callback)

expect_list_clear()
expect_list_extend([sim, sil, mavproxy])

# get a mavlink connection going
try:
	mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
except Exception, msg:
        print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
        raise
mav.message_hooks.append(message_hook)
mav.idle_hooks.append(idle_hook)

failed = False
failed_test_msg = "None"

###########Tests Sogilis###############
try:
	mav.wait_heartbeat()
        homeloc = mav.location()

	if len(sys.argv) > 0:
    		matched = []
    		for a in sys.argv:
        		for s in tests:	   
            			if fnmatch.fnmatch(s.lower(), a.lower()):
               				matched.append(s)
    
	if "arm_disarm" in matched :
        	res = tests_sogilis.arm_disarm(mavproxy,mav)
		display("arm_disarm", res) 

	if "failsafe" in matched :
        	res = tests_sogilis.failsafe(mavproxy,mav)
		display("failsafe", res)

	if "takeoff" in matched :
        	res = tests_sogilis.takeoff(mavproxy,mav)
		display("takeoff", res)
    
        if "guided_test" in matched :
            res = tests_sogilis.guided_test(mavproxy,mav)
            display("guided_test", res) 

        if "change_speed" in matched :
            res = tests_sogilis.change_speed(mavproxy,mav)
            display("change speed", res)

        if "change_speed2" in matched :
            res = tests_sogilis.change_speed2(mavproxy,mav)
            display("change speed with 2 thresholds", res)

        if "change_yaw" in matched :
            res = tests_sogilis.change_yaw(mavproxy,mav)
            display("change yaw", res)

        if "change_yaw2" in matched :
            res = tests_sogilis.change_yaw2(mavproxy,mav)
            display("change yaw (2 yaws)", res)

except pexpect.TIMEOUT, failed_test_msg:
        failed_test_msg = "Timeout"
        failed = True

mav.close()
util.pexpect_close(mavproxy)
util.pexpect_close(sil)
util.pexpect_close(sim)

if os.path.exists('ArduCopter-valgrind.log'):
        os.chmod('ArduCopter-valgrind.log', 0644)
        shutil.copy("ArduCopter-valgrind.log", util.reltopdir("../buildlogs/ArduCopter-valgrind.log"))

if failed:
        print("FAILED: %s" % failed_test_msg)

