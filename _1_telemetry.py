from pymavlink import mavutil

class TelemetryHandler:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        print("Connecting to SILT...")
        self.conn = mavutil.mavlink_connection(connection_string)
        self.conn.wait_heartbeat()
        print("Connected to SILT")

        self.data = {
            "lat": None,
            "lon": None,
            "alt": None,
            "ground_speed": None,
            "heading": None,
            "mode": None,
            "battery": None
        }

    def update(self):
        while True:
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                break
            
            msg_type = msg.get_type()

            if msg_type == "GLOBAL_POSITION_INT":
                self.data["lat"] = msg.lat / 1e7 # MAVLink sends In scaled integer and we divide to convert into degrees like 473667788 --> 47.3667788
                self.data["lon"] = msg.lon / 1e7
                self.data['alt'] = msg.relative_alt / 1000       #"relative_alt"->height from ground.

            elif msg_type == "VFR_HUD":
                self.data["ground_speed"] = msg.groundspeed 
                self.data["heading"] = msg.heading

            elif msg_type == "SYS_STATUS":
                self.data["battery"] = msg.battery_remaining

            elif msg_type == "HEARTBEAT":
                self.data["mode"] = mavutil.mode_string_v10(msg) #Mode:Guided,Auto,Stabilize. 
                

        return self.data




