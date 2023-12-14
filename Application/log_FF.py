import numpy as np
import matplotlib.pyplot as plt
import serial
from matplotlib import animation

dispNum = 200
x = np.linspace(0, 200, 201)

ser = serial.Serial('/dev/ttyUSB0', 115200)

# using the variable axs for multiple Axes
fig, ((plot_extPID, plot_inPID,), (plot_extPID_output, plot_inPID_output)) = plt.subplots(2, 2)

plot_extPID_line = [
	plot_extPID.plot([], [], ls='--', lw=1, label='tar', c='b'),
	plot_extPID.plot([], [], lw=1, label='ext_cur', c='r'),
]

plot_extPID_output_line = [
    plot_extPID_output.plot([], [], lw=1, label='p', c='r'),
    plot_extPID_output.plot([], [], lw=1, label='i', c='b'),
    plot_extPID_output.plot([], [], lw=1, label='d', c='g'),
    ] 

plot_inPID_line = [
	plot_inPID.plot([], [], ls='--', lw=1, label='tar', c='b'),
	plot_inPID.plot([], [], lw=1, label='in_cur', c='r'),
]

plot_inPID_output_line = [
    plot_inPID_output.plot([], [], lw=1, label='p', c='r'),
    plot_inPID_output.plot([], [], lw=1, label='i', c='b'),
    plot_inPID_output.plot([], [], lw=1, label='d', c='g')
    ]

plot_extPID.set_title('extPID')
plot_extPID.legend()
plot_extPID.set_ylabel(r'$DEG$')
plot_extPID.set_xlim((0, dispNum))
plot_extPID.set_ylim((-60, 60))

plot_extPID_output.set_title('extPID-output')
plot_extPID_output.legend()
plot_extPID_output.set_ylabel(r'$DEG/S$')
plot_extPID_output.set_xlim((0, dispNum))
plot_extPID_output.set_ylim((-40, 40))

plot_inPID.set_title('inPID')
plot_inPID.legend()
plot_inPID.set_ylabel(r'$DEG$')
plot_inPID.set_xlim((0, dispNum))
plot_inPID.set_ylim((-60, 60))

plot_inPID_output.set_title('inPID-output')
plot_inPID_output.legend()
plot_inPID_output.set_ylabel(r'$DEG/S$')
plot_inPID_output.set_xlim((0, dispNum))
plot_inPID_output.set_ylim((-40, 40))

extPID_data = [np.zeros(dispNum + 1) for _ in range(2)]
inPID_data = [np.zeros(dispNum + 1) for _ in range(2)]

extPID_out_data = [np.zeros(dispNum + 1) for _ in range(3)]
inPID_out_data = [np.zeros(dispNum + 1) for _ in range(3)]

def is_float(element: any) -> bool:
    # If you expect None to be passed:
    if element is None:
        return False
    try:
        float(element)
        return True
    except ValueError:
        return False

def WW():
    global extPID_data
    global extPID_out_data
    global inPID_data
    global inPID_out_data
    valid = False
    ser.flushInput()
    while (not valid):
        line1 = ser.readline()
        line1 = ser.readline()
        sl1 = str(line1, 'utf-8')

        t1 = sl1.split()
        if (len(t1) == 10):
            for i in range(0, 10):
                if (not is_float(t1[i])):
                    valid = False
                    break
                else:
                    valid = True
    
    # Generate random values for testing
    # t1 = [(str)(np.random.uniform(-30, 30)) for _ in range(10)]
    
    # append data to the end of the array(e.g. dispNum)
    extPID_data[0] = np.append(extPID_data[0], float(t1[0]))
    extPID_data[1] = np.append(extPID_data[1], float(t1[1]))
    
    extPID_out_data[0] = np.append(extPID_out_data[0], float(t1[2]))
    extPID_out_data[1] = np.append(extPID_out_data[1], float(t1[3]))
    extPID_out_data[2] = np.append(extPID_out_data[2], float(t1[4]))
    
    inPID_data[0] = np.append(inPID_data[0], float(t1[5]))
    inPID_data[1] = np.append(inPID_data[1], float(t1[6]))
    
    inPID_out_data[0] = np.append(inPID_out_data[0], float(t1[7]))
    inPID_out_data[1] = np.append(inPID_out_data[1], float(t1[8]))
    inPID_out_data[2] = np.append(inPID_out_data[2], float(t1[9]))
    
    # delete the first element
    extPID_data[0] = np.delete(extPID_data[0], 0)
    extPID_data[1] = np.delete(extPID_data[1], 0)
    
    extPID_out_data[0] = np.delete(extPID_out_data[0], 0)
    extPID_out_data[1] = np.delete(extPID_out_data[1], 0)
    extPID_out_data[2] = np.delete(extPID_out_data[2], 0)
    
    inPID_data[0] = np.delete(inPID_data[0], 0)
    inPID_data[1] = np.delete(inPID_data[1], 0)
    
    inPID_out_data[0] = np.delete(inPID_out_data[0], 0)
    inPID_out_data[1] = np.delete(inPID_out_data[1], 0)
    inPID_out_data[2] = np.delete(inPID_out_data[2], 0)

    return (extPID_data, extPID_out_data, inPID_data, inPID_out_data)

def update(t):
    global extPID_cur
    global extPID_tar
    global inPID_cur
    global extPID
    extPID_data, extPID_out_data, inPID_data, inPID_out_data = WW()
    plot_extPID_line[0][0].set_data(x, extPID_data[0])
    plot_extPID_line[1][0].set_data(x, extPID_data[1])
    plot_extPID_output_line[0][0].set_data(x, extPID_out_data[0])
    plot_extPID_output_line[1][0].set_data(x, extPID_out_data[1])
    plot_extPID_output_line[2][0].set_data(x, extPID_out_data[2])
    plot_inPID_line[0][0].set_data(x, inPID_data[0])
    plot_inPID_line[1][0].set_data(x, inPID_data[1])
    plot_inPID_output_line[0][0].set_data(x, inPID_out_data[0])
    plot_inPID_output_line[1][0].set_data(x, inPID_out_data[1])
    plot_inPID_output_line[2][0].set_data(x, inPID_out_data[2])
    
    plot_extPID.set_ylim(np.min(extPID_data) - 10, np.max(extPID_data) + 10)
    plot_extPID_output.set_ylim(np.min(extPID_out_data) - 10, np.max(extPID_out_data) + 10)
    plot_inPID.set_ylim(np.min(inPID_data) - 10, np.max(inPID_data) + 10)
    plot_inPID_output.set_ylim(np.min(inPID_out_data) - 10, np.max(inPID_out_data) + 10)

ani = animation.FuncAnimation(
    fig, update, fargs=(), interval=5)

plt.show()