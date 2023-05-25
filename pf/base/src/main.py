"""
Written by: Kalem Fostin, Hayden Bellingham and Gian Barta-Dougall, Brody Olabarriaga
DATE: 26/05/2023
Brief:
    This python file is used for reading radar data and processing it to both object
    tracking and occupancy counts. This will be sent to a web dashboard
"""
import threading as thread
import numpy as np
import matplotlib.animation as animation
from scipy.ndimage import gaussian_filter 
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
import pandas as pd
import queue, time, serial
from sklearn.cluster import DBSCAN
import math
import os
# Web Dashboard Imports
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import time
import serial


# InfluxDB connection details
token = "u2CZiySaIzkVukwhCJDIUGqPf7g97XHPlncnC5YGXGrsgU2YdM0owBVUTCSFSYbqIN6GVSbQUL8U__RjQOyW0g=="
org = "The University of Queensland"
RSSI_Bucket = "Static_Node_RSSI"
coordinate_bucket = "Float"
ml_bucket = "ML_Data"
url = "https://us-east-1-1.aws.cloud2.influxdata.com"
write_client = InfluxDBClient(url=url, token=token, org=org, verify_ssl=False)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Global Variables
xVals = []
yVals = []
ANI_FRAMES = 200
ANI_INTERVAL = 100
MAX_XPOS = 4
ANGLE = abs(math.tan(45 * (math.pi / 180.0)))
CLUSTER_SAMPLES = 20
CLUSTER_EPS = 0.15
FILTER_SIGMA = 0.35

radarDataQueue = queue.Queue(maxsize = 10) # queue used to send radar data over for processing
webQueue = queue.Queue(maxsize = 10)
cliPort = serial.Serial('COM9', 115200) # connect cli port
dataPort = serial.Serial('COM7', 921600) # connect data port
nrf = serial.Serial('COM10', 115200) # Connect to nrf dongle
configFileName = "AWR1843config.cfg"
byteBuffer = np.zeros(2**15, dtype = 'uint8')
byteBufferLength = 0
clusterPoints = pd.DataFrame() # empty data frame
numClusters = [0] * 3 # store the number of clusters
occupantsFound = 0

timeCounter = 0
newTime = 0

# Create the figure and subplots
radarFigure = plt.figure(figsize=(20, 6))
ax3d = radarFigure.add_subplot(121, projection='3d')
ax_xy = radarFigure.add_subplot(122)

# Set plot limits for 3D animation
ax3d.set_xlim([-MAX_XPOS, MAX_XPOS])
ax3d.set_ylim([0, 5])
ax3d.set_zlim([-4, 4])
ax3d.set_xlabel('X')
ax3d.set_ylabel('Y')
ax3d.set_zlabel('Z')

ax_xy.set_xlim([-MAX_XPOS, MAX_XPOS])
ax_xy.set_ylim([0, 5])
ax_xy.set_xlabel('X')
ax_xy.set_ylabel('Y')

occupantCluster = pd.DataFrame({'X': [0], 'Y': [0], 'Z': [0], 'clusterName': [0]}, index = [0]) 
occupant = pd.DataFrame({'X' : [], 'Y' : []})
# Create scatter plots for bees in 3D animation and XY occupantDimmensionsitions subplot
scatter_3d = ax3d.scatter(0, 0, 0, c='blue', marker='o')
scatter_xy = ax_xy.scatter(0, 0, c='black', marker='o')

# these values are used to store the box max and min values for each
occupantDimmensions = pd.DataFrame({'maxZ': [], 'minZ': [], 'center': [], 'minX': [], 'maxX': [], 'minY': [], 'maxY': []})
occupantBox = []


nrf.write(bytes('ble mac FE:78:2C:D5:CB:B8\n', 'utf-8'))
time.sleep(0.1)
nrf.write(bytes('ble scan start\n', 'utf-8'))
time.sleep(3)

TEST = 0
if TEST:
    def test():
        while True:
            nrf.write(bytes('num 0\n', 'utf-8'))
            time.sleep(1)
            print("Loop")


    t1 = thread.Thread(target=test)
    t1.start()

"""
serial_config()
    This function is used for writing the config file to the radar
    device
"""
def serial_config(configFileName):
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        cliPort.write((i + '\n').encode())
        print(i)
        time.sleep(0.01)
    return cliPort, dataPort

"""
parse_config_file()
    This function is usde for parsing the config file
"""
def parse_config_file(configFileName):
    configParameters = {} # stores config parameters
    
    # read config and send it to the radar
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        numRxAnt = 4 # Number of RX Antennas
        numTxAnt = 3 # Number of TX Antennas
        
        # get profile configuration information
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        # get frame configuration information  
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])
            
    # combine and read values          
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame // numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters


"""
read_radar_data()
    This function is used for reading the radar data. This dta will be returned
    as a dataframe containing the read x, y and z positions and the velocity
    of the object
"""
def read_radar_data(Dataport, configParameters):
    # Constants
    global byteBufferLength, byteBuffer
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    maxBufferSize = 2**15
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0 # counter representing the number of frames read
    detObj = {}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength : byteBufferLength + byteCount] = byteVec[ : byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all occupantDimmensionssible locations of the magic word
        occupantDimmensionssibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in occupantDimmensionssibleLocs:
            check = byteBuffer[loc : loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)
            
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0] : byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0] : ]), dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            totalPacketLen = np.matmul(byteBuffer[12 : 12 + 4], word)
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        # Initialize the pointer index
        idX = 0
        # Read the header
        magicNumber = byteBuffer[idX : idX + 8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX : idX + 4], word), 'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX : idX + 4], word), 'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX : idX + 4], word)
        idX += 4
        
        for tlvIdx in range(numTLVs):
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            tlv_type = np.matmul(byteBuffer[idX : idX + 4], word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX : idX + 4], word)
            idX += 4

            # Read x,y,z and velocity data 
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                x = np.zeros(numDetectedObj, dtype = np.float32)
                y = np.zeros(numDetectedObj, dtype = np.float32)
                z = np.zeros(numDetectedObj, dtype = np.float32)
                velocity = np.zeros(numDetectedObj, dtype = np.float32)
                
                for objectNum in range(numDetectedObj):
                    # Read the data for each object
                    x[objectNum] = byteBuffer[idX : idX + 4].view(dtype = np.float32)
                    idX += 4
                    y[objectNum] = byteBuffer[idX : idX + 4].view(dtype = np.float32)
                    idX += 4
                    z[objectNum] = byteBuffer[idX : idX + 4].view(dtype = np.float32)
                    idX += 4
                    velocity[objectNum] = byteBuffer[idX : idX + 4].view(dtype = np.float32)
                    idX += 4

                detObj = pd.DataFrame({"X" : x, "Y" : y , "Z" : z , "Velocity" : velocity})
                dataOK = 1
                
        # Remove already processed data
        if idX > 0 and byteBufferLength>idX:
            shiftSize = totalPacketLen
                    
            byteBuffer[ : byteBufferLength - shiftSize] = byteBuffer[shiftSize : byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize : ] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize : ]), dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            if byteBufferLength < 0:
                byteBufferLength = 0         
    return dataOK, frameNumber, detObj
    
"""
find_occupants()
    This function will cluster the data to represent individual objects. The mean x, y and z
    positions are calculated to represent the position of each object (cluster)
"""
def find_occupants(radarData, radarDataToBeParsed):
    timer = time.time()
    global occupantCluster, occupant, occupantDimmensions, xVals, yVals, timeCounter
    # center occupantDimmensionsition of the occupant after clustering
    occupantCluster = pd.DataFrame({'X': [0], 'Y': [0], 'Z': [0], 'clusterName': [0]}, index = [0]) 
    if len(radarDataToBeParsed) == 0: # if there is no data then return
        return 0

    radarDataDF = pd.DataFrame({'X': radarData[0, :], 'Y': radarData[1, :]})

    clusterDB = DBSCAN(eps = CLUSTER_EPS, min_samples = CLUSTER_SAMPLES).fit(radarDataDF) 
    clusterLabels = clusterDB.labels_

    # get all indices from every value that is not -1... i.e. all values that isnt considered noise
    indices = [count for count, value in enumerate(clusterLabels) if value != -1]
    clusterPoints = radarDataToBeParsed.loc[indices] # only get the indices that are valid from the cluster i.e. not noise
    clusterPoints['clusterName'] = [clusterLabels[index] for index in indices] 

    # Number of clusters in labels, ignoring noise if present.
    numClusters = len(set(clusterLabels)) - (1 if -1 in clusterLabels else 0)
  
    maxZVal = []
    minZVal = []
    centerVal = []
    minYVal = []
    maxYVal = []
    minXVal = []
    maxXVal = []

    for clusterLabel in range(numClusters):
        # Iterate through, assign each point to a cluster and find the centre point of each
            cluster = clusterPoints[clusterPoints['clusterName'] == clusterLabel]
            if len(cluster) < 50:
                continue
            centerPoint = np.mean(cluster, axis = 0)
            flag = 0
            append2 = 0

            if xVals and yVals: 
                for i in range(len(xVals)):
                    d = math.sqrt((centerPoint['X'] - xVals[i])**2 + (centerPoint['Y'] - yVals[i])**2)
                    if (d < 0.6): 
                        centerPoint['Y'] = 0.1 if centerPoint['Y'] == 0 else centerPoint['Y']
                        if abs((centerPoint['X'] / centerPoint['Y'])) > ANGLE:
                            del xVals[i]
                            del yVals[i]
                            flag = 1
                            break
                        xVals[i] = centerPoint['X']
                        yVals[i] = centerPoint['Y']
                        flag = 1
                        break
            else:
                xVals.append(centerPoint['X'])
                yVals.append(centerPoint['Y'])
                append2 = 1
            
            if flag == 0 and append2 == 0:
                xVals.append(centerPoint['X'])
                yVals.append(centerPoint['Y'])

            q1Z, q3Z = np.percentile(cluster['Z'], [25, 75])
            iqrZ = q3Z - q1Z
            maxZ = q3Z + (1.5 * iqrZ)
            minZ = q1Z - (1.5 * iqrZ)

            maxZVal.append(maxZ)
            minZVal.append(minZ)
            centerVal.append(centerPoint)
            maxXVal.append(max(cluster['X']))
            minXVal.append(min(cluster['X']))
            maxYVal.append(max(cluster['Y']))
            minYVal.append(min(cluster['Y']))

    occupantDimmensions = pd.DataFrame({'maxZ': maxZVal, 'minZ': minZVal, 'center': centerVal, 'minX': minXVal, 'maxX': maxXVal, 'minY': minYVal, 'maxY': maxYVal})

    occupantCluster = clusterPoints
    occupant = pd.DataFrame({'X': xVals, 'Y': yVals})
    webQueue.put(occupant)

"""
filter_noise()
    This function is used to filter out noise from the radar.
    The data to be filtered is enterred and the filtered array is returned.

"""
def filter_noise(data):
    radarData = data.copy()
    radarData = np.vstack((radarData['X'], radarData['Y']))
    return gaussian_filter(radarData, sigma = FILTER_SIGMA) # quieten noise

def create_box_for_objects(occupantDimmensions):
    # Remove previous boxes 
    for box in occupantBox:
        box.remove()
    occupantBox.clear()

    # create lines and vertices for each box around each object
    for i in range(len(occupantDimmensions['maxX'])):
        minX = occupantDimmensions['minX'].loc[i]
        maxX = occupantDimmensions['maxX'].loc[i]
        minY = occupantDimmensions['minY'].loc[i]
        maxY = occupantDimmensions['maxY'].loc[i]
        minZ = occupantDimmensions['minZ'].loc[i]
        maxZ = occupantDimmensions['maxZ'].loc[i]

        # each point of each box
        box = np.array([
            [minX, minY, maxZ], [minX, maxY, maxZ],
            [maxX, minY, maxZ], [maxX, maxY, maxZ],
            [minX, minY, minZ], [minX, maxY, minZ],
            [maxX, minY, minZ], [maxX, maxY, minZ],])
        
        # Lines to connect each vertice
        lines = [[box[0], box[2], box[3], box[1], box[0]],
                [box[4], box[6], box[7], box[5], box[4]],
                [box[4], box[0]],[box[2], box[6]],
                [box[3], box[7]],[box[1], box[5]]]

        line_collection = Line3DCollection(lines, colors = 'red')
        ax3d.add_collection3d(line_collection)
        occupantBox.append(line_collection)

    return occupantBox

"""
update_plots()
    This function will update the tracking on a XY plot and the position of the object
    in a 3D plot. A box is also drawn around the objects in the 3d spot with the box size
    being relative to the dimmensions of the object
"""
def update_plots(frame):
    global occupant, occupantDimmensions, occupantCluster, occupantsFound

    scatter_3d._offsets3d = (occupantCluster['X'], occupantCluster['Y'], occupantCluster['Z'])
    ax_xy.set_title('Occupant Count: %i' % len(occupant)) 
    scatter_xy.set_offsets(occupant)
    occupantBoxes = create_box_for_objects(occupantDimmensions)

    if len(occupant) > 0:
        if occupantsFound == 0:
            occupantsFound = 1
            # Toggle led
            nrf.write(bytes('num 0\n', 'utf-8'))
    else:
        if occupantsFound > 0:
            # Toggle led
            occupantsFound = 0
            nrf.write(bytes('num 0\n', 'utf-8'))

    return scatter_3d , scatter_xy, *occupantBoxes

"""
process_radar()
    This function will continuously get the data read from the radar and
    process the data for determining occupancy and tracking.
"""
def process_radar():
    global timeCounter, newTime
    # continuously loop to process radar data
    while True:
        radarData = radarDataQueue.get()
        radarData = radarData.dropna()
        filteredRadarData = filter_noise(radarData)
        timer = time.time()
        newTime = timer - timeCounter
        timeCounter = timer
        find_occupants(filteredRadarData, radarData)

"""
send_to_dashboard()
    This function is used to send the occupant data to the webdashboard
"""
def send_to_dashboard():
    while True:
        dataL = webQueue.get()
        coordinate_data = {"coordinate_data": []}
        for i in range(len(dataL)):
            coordinate_data["coordinate_data"].append({"id": i, "x_coordinate": dataL["X"].loc[i], "y_coordinate": dataL["Y"].loc[i]})
        
        coordinate_bucket_points = []
        for data in coordinate_data["coordinate_data"]:
            point = (
                Point("coordinate_system")
                .tag("data_type", "coordinate_data")
                .tag("id", str(data["id"]))
                .field("x_coordinate", data["x_coordinate"])
                .field("y_coordinate", data["y_coordinate"])
            )
            coordinate_bucket_points.append(point)

        write_api.write(bucket=coordinate_bucket, org=org, record=coordinate_bucket_points)
        time.sleep(0.01) 

"""
determine_occupancy()
    This function will be used for initialising the radar and reading
    the x,y and z positions and velocity from the radar. These values are
    then sent over a queue for processing.
"""
def determine_occupancy():
    # Get the configuration parameters from the configuration file
    serial_config(configFileName)
    configParameters = parse_config_file(configFileName)
    # continuously loop to determine the occupancy of the room
    while True:
        # Read and parse the received data
        dataOk, frameNumber, detObj = read_radar_data(dataPort, configParameters)

        if dataOk and len(detObj['Velocity']): # if only we received a complete message and detected a large num of points
            radarDataQueue.put(detObj) # send the data to the 
        time.sleep(0.01) # Sampling frequency of 30 Hz

"""
Main, used to initialising the threads and animation
"""
if __name__ == "__main__":
    try:
        thread.Thread(target = determine_occupancy).start() # thread to read radar
        thread.Thread(target = process_radar).start() # thread to process the radar data
        thread.Thread(target = send_to_dashboard).start() # thread to send processed data to web dashboard

        # Start the animation
        trackingAnimation = animation.FuncAnimation(radarFigure, update_plots, frames = ANI_FRAMES, interval = ANI_INTERVAL)
        plt.show()
    except KeyboardInterrupt:
        cliPort.write(('sensorStop\n').encode())
        cliPort.close()
        dataPort.close()
        trackingAnimation.event_source.stop()
        plt.close(radarFigure)
        print("RADAR IS NOT OPERATIONAL :(")
        os._exit(1)