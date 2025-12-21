import zmq
import json
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import numpy as np

# ZMQ Setup
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect("tcp://localhost:5555") # Connect to WSL bridge
sock.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all

# Data Buffers
buffer_size = 500
data_ax = np.zeros(buffer_size)
data_ay = np.zeros(buffer_size)
data_az = np.zeros(buffer_size)
ptr = 0

# PyQtGraph App
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="ALTAIR Real-time Stream")
win.resize(1000, 600)
win.setWindowTitle('ALTAIR Decoupled GCS (Native Windows)')

# Plot 1: Accel
p1 = win.addPlot(title="Linear Acceleration (m/s^2)")
curve_ax = p1.plot(pen='r', name='Ax')
curve_ay = p1.plot(pen='g', name='Ay')
curve_az = p1.plot(pen='b', name='Az')
p1.addLegend()

win.nextRow()

# Plot 2: Gyro
p2 = win.addPlot(title="Angular Velocity (rad/s)")
curve_gx = p2.plot(pen='r', name='Gx')
curve_gy = p2.plot(pen='g', name='Gy')
curve_gz = p2.plot(pen='b', name='Gz')

# Data storage for gyro
data_gx = np.zeros(buffer_size)
data_gy = np.zeros(buffer_size)
data_gz = np.zeros(buffer_size)

def update():
    global ptr, data_ax, data_ay, data_az, data_gx, data_gy, data_gz
    
    # Read all pending messages
    try:
        while True:
            msg = sock.recv_string(flags=zmq.NOBLOCK)
            packet = json.loads(msg)
            
            if packet['topic'] == 'imu':
                d = packet['data']
                # Shift buffer
                data_ax[:-1] = data_ax[1:]
                data_ay[:-1] = data_ay[1:]
                data_az[:-1] = data_az[1:]
                
                data_gx[:-1] = data_gx[1:]
                data_gy[:-1] = data_gy[1:]
                data_gz[:-1] = data_gz[1:]
                
                # New data
                data_ax[-1] = d['ax']
                data_ay[-1] = d['ay']
                data_az[-1] = d['az']
                
                data_gx[-1] = d['gx']
                data_gy[-1] = d['gy']
                data_gz[-1] = d['gz']
                
                ptr += 1
    except zmq.Again:
        pass # No more data

    # Update curves
    curve_ax.setData(data_ax)
    curve_ay.setData(data_ay)
    curve_az.setData(data_az)
    
    curve_gx.setData(data_gx)
    curve_gy.setData(data_gy)
    curve_gz.setData(data_gz)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(16) # 60 FPS

if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
