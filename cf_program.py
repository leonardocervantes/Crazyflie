
#!/usr/bin/python3

"""
GUI for controlling Crazyflie. Inputs a height between .8 and 2m and hovers
at that height using a PID. This is also possible throught the motion commander.

"""

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

from tkinter import *


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


# def init(data, cf, scf):
#     data.cf = cf
#     data.scf = scf
#     data.mc = None

# def start_Motion_Commander(data):
#     print("Starting Motion Commander ...")
#     mc = MotionCommander(data.scf)
#     time.sleep(1)
#     data.mc = mc

def scanAction(w):
    print("Scanning interfaces for Crazyflies...")
    w.uri_available = cflib.crtp.scan_interfaces()

    print(w.uri_available)
    if (len(w.uri_available) > 0):
        print("Crazyflies found:")

        w.uriDropdown.children["menu"].delete(0,"end")
        for i in w.uri_available:
            # w.uriVar.set(i[0])
            w.uriDropdown.children["menu"].add_command(label = i[0], command=lambda  uri = i[0]: w.uriVar.set(uri))
            print(i[0])

        w.uriVar.set(w.uri_available[0][0])
    else:
        w.uriVar.set("");
        print("No Crazyflie Found!")

def connectAction(w, drone):

    link_uri = w.uriVar.get();
    if (link_uri != ""):
        drone.connect(link_uri);
    else:
        print("Need URI to connect!")

class State():
    def __init__(self):
        x = 0;
        y = 0;
        z = 0;
        vx = 0;
        vy = 0;
        vz = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;
        roll_rate = 0;
        pitch_rate = 0;
        yaw_rate = 0;
        thrust = 0;

class Drone():
    def __init__(self):
        """ Initialize and run the example with the specified link_uri """

        self.state = State();
        self.cf = Crazyflie()
        self.is_connected = False;

        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)


    def connect(self, link_uri):
        print('Connecting to %s' % link_uri)
        self.cf.open_link(link_uri);
        self.is_connected = True

    def disconnect(self):
        self.cf.close_link()

    def connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded. """
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        # State Estimation from TOC
        self.lg_state = LogConfig(name='stateEstimate', period_in_ms=10)
        self.lg_state.add_variable('stateEstimate.x', 'float')
        self.lg_state.add_variable('stateEstimate.y', 'float')
        self.lg_state.add_variable('stateEstimate.z', 'float')

        self.lg_state_vel = LogConfig(name='stateVelEstimate', period_in_ms=10)
        self.lg_state_vel.add_variable('stateEstimate.vx', 'float')
        self.lg_state_vel.add_variable('stateEstimate.vy', 'float')
        self.lg_state_vel.add_variable('stateEstimate.vz', 'float')

        self.lg_euler = LogConfig(name='eulerEstimate', period_in_ms=10)
        self.lg_euler.add_variable('stabilizer.roll', 'float')
        self.lg_euler.add_variable('stabilizer.pitch', 'float')
        self.lg_euler.add_variable('stabilizer.yaw', 'float')
        self.lg_euler.add_variable('stabilizer.thrust', 'float')

        self.lg_euler_rate = LogConfig(name='eulerEstimate', period_in_ms=10)
        self.lg_euler_rate.add_variable('controller.rollRate', 'float')
        self.lg_euler_rate.add_variable('controller.pitchRate', 'float')
        self.lg_euler_rate.add_variable('controller.yawRate', 'float')


        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self.cf.log.add_config(self.lg_state)
            self.cf.log.add_config(self.lg_state_vel)
            self.cf.log.add_config(self.lg_euler)
            self.cf.log.add_config(self.lg_euler_rate)

            # This callback will receive the data
            self.lg_state.data_received_cb.add_callback(self.state_log_data)
            self.lg_state_vel.data_received_cb.add_callback(self.state_vel_log_data)
            self.lg_euler.data_received_cb.add_callback(self.euler_log_data)
            self.lg_euler_rate.data_received_cb.add_callback(self.euler_rate_log_data)

            # This callback will be called on errors
            self.lg_state.error_cb.add_callback(self.log_error)
            self.lg_state_vel.error_cb.add_callback(self.log_error)
            self.lg_euler.error_cb.add_callback(self.log_error)
            self.lg_euler_rate.error_cb.add_callback(self.log_error)

            # Start the logging
            self.lg_state.start()
            self.lg_state_vel.start()
            self.lg_euler.start()
            self.lg_euler_rate.start()

        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError as e:
            print(e)
            print('Could not add StateEstimate log config, bad configuration.')


    def log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))



    def state_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        self.state.x = float(data.get('stateEstimate.x'))
        self.state.y = float(data.get('stateEstimate.y'))
        self.state.z = float(data.get('stateEstimate.z'))
        # print("z: {:2.4}".format(self.state.z))

    def state_vel_log_data(self, timestamp, data, logconf):
        self.state.vx = float(data.get('stateEstimate.vy'))
        self.state.vy = float(data.get('stateEstimate.vy'))
        self.state.vz = float(data.get('stateEstimate.vz'))
        # print("vx: {:4.4}, vy: {:4.4}, vz: {:4.4}".format(self.state.vx,self.state.vy,
        #     self.state.vz))

    def euler_log_data(self, timestamp, data, logconf):
        self.state.roll = float(data.get('stabilizer.roll'))
        self.state.pitch = float(data.get('stabilizer.pitch'))
        self.state.yaw = float(data.get('stabilizer.yaw'))
        self.state.thrust = int(data.get('stabilizer.thrust'))
        # print("roll: {}, pitch: {}, yaw: {}".format(self.state.roll,self.state.pitch,
        #     self.state.yaw))

    def euler_rate_log_data(self, timestamp, data, logconfig):
        self.state.roll_rate = float(data.get("controller.rollRate"))
        self.state.pitch_rate = float(data.get("controller.pitchRate"))
        self.state.yaw_rate = float(data.get("controller.yawRate"))
        # print("roll rate: {}, pitch rate: {}, yaw rate: {}".format(self.state.roll_rate,self.state.pitch_rate,
        #     self.state.yaw_rate))


    def connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

def goAction(w, drone):
    # P = float(w.PText.get());
    # I = float(w.IText.get());
    #  D = float(w.DText.get());

    drone.cf.param.set_value('motorPowerSet.enable','1');

    drone.cf.param.set_value('motorPowerSet.m1','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m2','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m3','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m4','{}'.format(0));
    time.sleep(.001);



    pwd = 1000;

    while (pwd < 10000):
        drone.cf.param.set_value('motorPowerSet.m1','{}'.format(pwd));
        time.sleep(.01);
        drone.cf.param.set_value('motorPowerSet.m2','{}'.format(pwd));
        time.sleep(.01);
        drone.cf.param.set_value('motorPowerSet.m3','{}'.format(pwd));
        time.sleep(.01);
        drone.cf.param.set_value('motorPowerSet.m4','{}'.format(pwd));
        time.sleep(.01);

        pwd += 100;
        print(pwd)
        time.sleep(.01);

    drone.cf.param.set_value('motorPowerSet.m1','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m2','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m3','{}'.format(0));
    time.sleep(.001);
    drone.cf.param.set_value('motorPowerSet.m4','{}'.format(0));
    time.sleep(.001);





    # targetHeight = float(w.heightText.get());
    # currHeight = drone.state.z;
    # currThrust = drone.state.thrust;
    # error = targetHeight - currHeight;
    # cum_error = 0;
    # print(currThrust)
    # drone.cf.commander.send_setpoint(0, 0, 0, 0)

    # if (currThrust < 2000):
    # 	currThrust = 2000
    # newThrust = 0;
    # while (error > 0.01):
    #     cum_error = cum_error + error;
    #     # print("error {%}\n", error)
    #     newThrust = newThrust + int(P * error);
    #     drone.cf.commander.send_setpoint(0,0,0, newThrust )
    #     currHeight = drone.state.z;
    #     currThrust = drone.state.thrust;
    #     error = targetHeight - currHeight;
    #     print("thrust {%}\n", newThrust)
    #     time.sleep(.01);


class createWindow():
    def __init__(self, master):
        # Create main window
        self.master = master
        self.uri_available = None
        # Create URI, dropdown and connect Buttons
        self.connectButton = None
        self.scanButton = None
        self.uriVar = None
        self.uriDropdown = None

        # Create PID Controller, height, go and stop Buttons
        self.controlFrame = None
        self.Ptext = None
        self.IText = None
        self.DText = None
        self.heightText = None
        self.goButton = None
        self.stopButton = None

        #create State Frame
        self.xVar = None
        self.yVar = None
        self.zVar = None
        self.vxVar = None
        self.vyVar = None
        self.vzVar = None
        self.rollVar = None
        self.pitchVar = None
        self.yawVar = None
        self.thrustVar = None
        self.stateFrame = None
        self.xText = None
        self.yText = None
        self.zText = None
        self.vxText = None
        self.vyText = None
        self.vzText = None
        self.rollText = None
        self.pitchText = None
        self.yawText = None
        self.thrustText = None

        self.createConnectFrame()
        self.createControlFrame()
        self.createStateFrame()

    def createStateFrame(self):
        self.stateFrame = Frame(self.master)
        xLabel = Label(self.stateFrame, text="x:")
        yLabel = Label(self.stateFrame, text="y:")
        zLabel = Label(self.stateFrame, text="z:")
        vxLabel = Label(self.stateFrame, text="vx:")
        vyLabel = Label(self.stateFrame, text="vy:")
        vzLabel = Label(self.stateFrame, text="vz:")
        rollLabel = Label(self.stateFrame, text="roll:")
        pitchLabel = Label(self.stateFrame, text="pitch:")
        yawLabel = Label(self.stateFrame, text="yaw:")
        thrustLabel = Label(self.stateFrame, text="thrust:")

        self.xVar = StringVar();
        self.yVar = StringVar();
        self.zVar = StringVar();
        self.vxVar = StringVar();
        self.vyVar = StringVar();
        self.vzVar = StringVar();
        self.rollVar = StringVar();
        self.pitchVar = StringVar();
        self.yawVar = StringVar();
        self.thrustVar = StringVar();

        self.xText = Entry(self.stateFrame,state=DISABLED, textvariable= self.xVar, width = 10)
        self.yText = Entry(self.stateFrame,state=DISABLED, textvariable= self.yVar, width = 10)
        self.zText = Entry(self.stateFrame,state=DISABLED, textvariable= self.zVar, width = 10)
        self.vxText = Entry(self.stateFrame,state=DISABLED, textvariable= self.vxVar, width = 10)
        self.vyText = Entry(self.stateFrame,state=DISABLED, textvariable= self.vyVar, width = 10)
        self.vzText = Entry(self.stateFrame,state=DISABLED, textvariable= self.vzVar, width = 10)
        self.rollText = Entry(self.stateFrame,state=DISABLED, textvariable= self.rollVar, width = 10)
        self.pitchText = Entry(self.stateFrame,state=DISABLED, textvariable= self.pitchVar, width = 10)
        self.yawText = Entry(self.stateFrame,state=DISABLED, textvariable= self.yawVar, width = 10)
        self.thrustText = Entry(self.stateFrame,state=DISABLED, textvariable= self.thrustVar, width = 10)

        xLabel.grid(row=0, column=0,sticky="E")
        yLabel.grid(row=1, column=0,sticky="E")
        zLabel.grid(row=2, column=0,sticky="E")
        vxLabel.grid(row=0, column=2,sticky="E")
        vyLabel.grid(row=1, column=2,sticky="E")
        vzLabel.grid(row=2, column=2,sticky="E")
        rollLabel.grid(row=3, column=0,sticky="E")
        pitchLabel.grid(row=4, column=0,sticky="E")
        yawLabel.grid(row=5, column=0,sticky="E")
        thrustLabel.grid(row=6, column=0,sticky="E")

        self.xText.grid(row=0, column=1)
        self.yText.grid(row=1, column=1)
        self.zText.grid(row=2, column=1)
        self.vxText.grid(row=0, column=3)
        self.vyText.grid(row=1, column=3)
        self.vzText.grid(row=2, column=3)
        self.rollText.grid(row=3, column=1)
        self.pitchText.grid(row=4, column=1)
        self.yawText.grid(row=5, column=1)
        self.thrustText.grid(row=6, column=1)

        self.stateFrame.pack(side = RIGHT)



    def createControlFrame(self):
        self.controlFrame = Frame(self.master)
        PLabel = Label(self.controlFrame,text = "P:");
        ILabel = Label(self.controlFrame,text = "I:");
        DLabel = Label(self.controlFrame,text = "D:");
        heightLabel = Label(self.controlFrame,text = "Height:")

        self.PText = Entry(self.controlFrame)
        self.IText = Entry(self.controlFrame)
        self.DText = Entry(self.controlFrame)
        self.heightText = Entry(self.controlFrame)

        self.goButton = Button(self.controlFrame,text = "Go")
        self.stopButton = Button(self.controlFrame,text = "Stop")

        PLabel.grid(row=0, column=0,sticky="E")
        ILabel.grid(row=1, column=0,sticky="E")
        DLabel.grid(row=2, column=0,sticky="E")
        heightLabel.grid(row=3, column = 0,sticky="E")

        self.PText.grid(row=0, column=1)
        self.IText.grid(row=1, column=1)
        self.DText.grid(row=2, column=1)
        self.heightText.grid(row=3, column = 1)

        self.goButton.grid(row=4, column = 0)
        self.stopButton.grid(row=4, column = 1)

        self.controlFrame.pack(side = LEFT)


    def createConnectFrame(self):

        self.connectFrame = Frame(self.master)
        uriLabel = Label(self.connectFrame, text = "URI :")
        self.connectButton = Button(self.connectFrame, text = "Connect")
        self.scanButton = Button(self.connectFrame, text = "Scan")

        self.uriVar = StringVar(self.connectFrame)
        self.uriVar.set("") # default value
        self.uriDropdown = OptionMenu(self.connectFrame, self.uriVar, self.uri_available)
        self.uriDropdown.config(width=15)
        uriLabel.pack(side=LEFT)
        self.uriDropdown.pack(side=LEFT)
        self.refreshButton.pack(side=LEFT)
        self.connectButton.pack(side=LEFT)

        self.connectFrame.pack(side=TOP)


if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # cf = Crazyflie(rw_cache='./cache')

    root = Tk()
    root.geometry('525x300')
    drone = Drone();
    window = createWindow(root)
    window.scanButton.bind("<Button-1>", lambda event: scanAction(w=window))
    window.connectButton.bind("<Button-1>", lambda event: connectAction(w = window, drone = drone))
    window.goButton.bind("<Button-1>", lambda event:goAction(w = window, drone = drone))

    root.mainloop()  # blocks until window is closed

    if (drone.is_connected == True):
        drone.disconnect();
