import sys
import os
import pymavlink
from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink


class MavlinkConnect:
    def __init__(self, portName, baudRate=230400):
        self.init_logger()
        self.TryConnectMavlink(portName, baudRate)
    
    def init_logger(self):
        self.init_compose_logger()
        # self.log_folder_name = "logs/{}".format(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
        # os.makedirs(self.log_folder_name, exist_ok=True)

    
    def init_compose_logger(self):
        compose_log_data_list = [
            "time_boot_ms",
            "BASE_MODE",
            "SYS_STATUS",
            "battery_voltage",
            "battery_current",
            "pwm[1]",
            "pwm[2]",
            "pwm[3]",
            "pwm[4]",
            "pwm[5]",
            "pwm[6]",
            "pwm[7]",
            "pwm[8]",
            "ch[1]",
            "ch[2]",
            "ch[3]",
            "ch[4]",
            "ch[5]",
            "ch[6]",
            "ch[7]",
            "ch[8]",
            "GPS_ALT",
            "PRESSURE_ALT",
            "ROLL_SP",
            "ROLL_EST",
            "PITCH_SP",
            "PITCH_EST",
            "YAW_SP",
            "YAW_EST",
            "ROLLRATE_SP",
            "ROLLRATE_EST",
            "PITCHRATE_SP",
            "PITCHRATE_EST",
            "YAWRATE_SP",
            "YAWRATE_EST",
            "AIRSPD",
            "GNDSPD_X",
            "GNDSPD_Y",
            "GNDSPD_Z",
            "GNDSPD_SP_X",
            "GNDSPD_SP_Y",
            "GNDSPD_SP_Z",
            "GNDSPD_XYNORM",
            "LOCALPOS_NED_EST_X",
            "LOCALPOS_NED_SP_X",
            "LOCALPOS_NED_EST_Y",
            "LOCALPOS_NED_SP_Y",
            "LOCALPOS_NED_EST_Z",
            "LOCALPOS_NED_SP_Z"
        ]
        self.compose_data_list = compose_log_data_list
        # self.compose_logger = open("{}/Compose_a2g.csv".format(self.log_folder_name), 'w')

        title = ", ".join(self.compose_data_list) + "\n"
        self.compose_log_data = dict()

    

    def TryConnectMavlink(self, portName, baudRate):
        self.master = mavutil.mavlink_connection(portName, baud=baudRate)


    def fast_nonblock_listen(self):
        if self.master is None:
            return
        self.try_receive_data_from_px4()


    def try_receive_data_from_px4(self, blocking=False):
        if self.master is None:
            return
        msg = None
        msg = self.master.recv_match(blocking=blocking)
        if msg is None:
            return

        if msg.get_type() != "BAD_DATA":

                # self.gcs_proxy.sendMsg(msg)
                # self.a2g_data_logger.info(msg)
                # self.a2g_csv_logger.process_msg(msg)

                self.process_compose_logger(msg)
            #     if msg.get_type() == "HEARTBEAT":
            #         self.process_a2g_heartbeat()

            #     self.count_buf_size += len(msg.get_msgbuf()) * 8
            # elif msg.get_type == "BAD_DATA":
            #     self.count_buf_size += len(msg._msgbuf) * 8
        


    def process_compose_logger(self, msg):
        if hasattr(msg, 'time_boot_ms'):
            self.compose_log_data['time_boot_ms'] = msg.time_boot_ms

        if msg.get_type() == "HEARTBEAT":
            self.compose_log_data["BASE_MODE"] = msg.base_mode
            print("BASE_MODE", self.compose_log_data["BASE_MODE"])
            self.compose_log_data["SYSTEM_STATUS"] = msg.system_status
            print("SYSTEM_STATUS",self.compose_log_data["SYSTEM_STATUS"])

        if msg.get_type() == "LOCAL_POSITION_NED":
            self.compose_log_data["LOCALPOS_NED_EST_X"] = msg.x
            # print "pos_x", self.compose_log_data["LOCALPOS_NED_EST_X"]
            self.compose_log_data["LOCALPOS_NED_EST_Y"] = msg.y
            self.compose_log_data["LOCALPOS_NED_EST_Z"] = msg.z

            self.compose_log_data["GNDSPD_X"] = msg.vx
            self.compose_log_data["GNDSPD_Y"] = msg.vy
            self.compose_log_data["GNDSPD_Z"] = msg.vz        

def main():
    
    mavc = MavlinkConnect("COM3")
    while True:
        mavc.fast_nonblock_listen()            

if __name__ == "__main__":
    os.environ['MAVLINK20'] = '1'
    main()

