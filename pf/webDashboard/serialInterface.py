import influxdb_client
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import time, pickle, os
import serial as serial_port

# InfluxDB connection details
token = "u2CZiySaIzkVukwhCJDIUGqPf7g97XHPlncnC5YGXGrsgU2YdM0owBVUTCSFSYbqIN6GVSbQUL8U__RjQOyW0g=="
org = "The University of Queensland"
RSSI_Bucket = "Static_Node_RSSI"
coordinate_bucket = "X_Y_Positional"
ml_bucket = "ML_Data"
url = "https://us-east-1-1.aws.cloud2.influxdata.com"
write_client = InfluxDBClient(url=url, token=token, org=org, verify_ssl=False)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

#Load in the ML model
file_path = os.path.expanduser(f'~/csse4011/repo/prac3/webDashboard/knn_model')
training_model = pickle.load(open(file_path, 'rb'))

# Serial Port Details
SERIAL_PORT_NAME = '/dev/tty.usbmodem141101'
BAUD_RATE = 115200
serial_port= serial_port.Serial(SERIAL_PORT_NAME, BAUD_RATE)


# Sample data
nodes_data = {
    "4011-A": {"rssi": 0},
    "4011-B": {"rssi": 0},
    "4011-C": {"rssi": 0},
    "4011-D": {"rssi": 0},
    "4011-E": {"rssi": 0},
    "4011-F": {"rssi": 0},
    "4011-G": {"rssi": 0},
    "4011-H": {"rssi": 0},
    # "4011-I": {"rssi": 0},
    # "4011-J": {"rssi": 0},
    # "4011-K": {"rssi": 0},
    # "4011-L": {"rssi": 0},
}
# Sample Coordinate Data
coordinate_data = {
    "coordinate_data" : {"x_coordinate" : 0, "y_coordinate" : 0},
}


while True:

    read_line = serial_port.readline().decode("utf-8").strip().split("|")
    node_names = ["4011-A", "4011-B", "4011-C", "4011-D", "4011-E", "4011-F", "4011-G", "4011-H"]
                #   "4011-I", "4011-J", "4011-K", "4011-L"]
    if len(read_line) >= 8:
        nodes_data["4011-A"]["rssi"] = -1 * (int(read_line[0][-2]) * 10 + int(read_line[0][-1]))
        nodes_data["4011-B"]["rssi"] = int(read_line[1])
        nodes_data["4011-C"]["rssi"] = int(read_line[2])
        nodes_data["4011-D"]["rssi"] = int(read_line[3])
        nodes_data["4011-E"]["rssi"] = int(read_line[4])
        nodes_data["4011-F"]["rssi"] = int(read_line[5])
        nodes_data["4011-G"]["rssi"] = int(read_line[6])
        nodes_data["4011-H"]["rssi"] = int(read_line[7])
        # nodes_data["4011-I"]["rssi"] = int(read_line[8])
        # nodes_data["4011-J"]["rssi"] = int(read_line[9])
        # nodes_data["4011-K"]["rssi"] = int(read_line[10])
        # nodes_data["4011-L"]["rssi"] = int(read_line[11])
    current_rssi_values = [[float(nodes_data[static_node]["rssi"]) for static_node in nodes_data]]
    # print(ml_data)
    ml_data_point = {}
    ml_data_point["ml_data_point"] = {}
    ml_data_point["ml_data_point"]["rssi1"] = -66
    ml_data_point["ml_data_point"]["rssi2"] = -56
    ml_data_point["ml_data_point"]["rssi3"] = -61
    ml_data_point["ml_data_point"]["rssi4"] = -55
    ml_data_point["ml_data_point"]["rssi5"] = -42
    ml_data_point["ml_data_point"]["rssi6"] = -66
    ml_data_point["ml_data_point"]["rssi7"] = -62
    ml_data_point["ml_data_point"]["rssi8"] = -61
    ml_data_point["ml_data_point"]["x_pos"] = 3.5
    ml_data_point["ml_data_point"]["y_pos"] = 4
    ml_data_point = Point("ml_data").field("rssi1", ml_data_point["ml_data_point"]["rssi1"]) \
    .field("rssi2", ml_data_point["ml_data_point"]["rssi2"]) \
    .field("rssi3", ml_data_point["ml_data_point"]["rssi3"]) \
    .field("rssi4", ml_data_point["ml_data_point"]["rssi4"]) \
    .field("rssi5", ml_data_point["ml_data_point"]["rssi5"]) \
    .field("rssi6", ml_data_point["ml_data_point"]["rssi6"]) \
    .field("rssi7", ml_data_point["ml_data_point"]["rssi7"]) \
    .field("rssi8", ml_data_point["ml_data_point"]["rssi8"]) \
    .field("x_pos", ml_data_point["ml_data_point"]["x_pos"]) \
    .field("y_pos", ml_data_point["ml_data_point"]["y_pos"])
    rssi_bucket_points = [Point("ml_data_point").tag("ml_data_point", static_node).field("rssi", rssi["rssi"])
                        for static_node, rssi in nodes_data.items()]
    write_api.write(bucket=ml_bucket, org=org, record=ml_data_point)

    rssi_bucket_points = [Point("node_data").tag("static_node", static_node).field("rssi", rssi["rssi"])
                        for static_node, rssi in nodes_data.items()]
    write_api.write(bucket=RSSI_Bucket, org=org, record=rssi_bucket_points)


    x, y = training_model.predict(current_rssi_values)[0]
    coordinate_data["coordinate_data"]["x_coordinate"] = x
    coordinate_data["coordinate_data"]["y_coordinate"] = y

    coordinate_bucket_points = Point("coordinate_system").tag("data_type", "coordinate_data").field("x_coordinate", coordinate_data["coordinate_data"]["x_coordinate"]).field("y_coordinate", coordinate_data["coordinate_data"]["y_coordinate"])

    write_api.write(bucket=coordinate_bucket, org=org, record=coordinate_bucket_points)


    # print(f"Node A: {nodes_data['4011-A']['rssi']}")
    # print(f"Node B: {nodes_data['4011-B']['rssi']}")
    # print(f"Node C: {nodes_data['4011-C']['rssi']}")
    # print(f"Node D: {nodes_data['4011-D']['rssi']}")
    # print(f"Node E: {nodes_data['4011-E']['rssi']}")
    # print(f"Node F: {nodes_data['4011-F']['rssi']}")
    # print(f"Node G: {nodes_data['4011-G']['rssi']}")
    # print(f"Node H: {nodes_data['4011-H']['rssi']}")
    # print(f"X: {coordinate_data['coordinate_data']['x_coordinate']}")
    # print(f"Y: {coordinate_data['coordinate_data']['y_coordinate']}")

    time.sleep(1)
