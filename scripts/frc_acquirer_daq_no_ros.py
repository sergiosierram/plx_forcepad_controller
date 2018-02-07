import time
from threading import Thread
from sys import stdin
from daq_controller.DAQ import DAQ

class FrcAcquirer():
    def __init__(self):
        self.frc_left = {'x':None,'y':None,'z':None}
        self.frc_right = {'x':None,'y':None,'z':None}
        self.exitFlag = False
        self.fs = 20
        self.daq = DAQ()
        self.x_forces = False
        self.dataExport = [True, 'data.txt']

    def acquire(self):
        if self.dataExport[0]:
            f = open(self.dataExport[1],'w')
            f.write("Time Stamp(s)\tFLy\tFLz\tFRy\tFRz\n")
            ti = time.time()
        print("Acquiring Data...")
        while not self.exitFlag:
            data = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]
            if self.x_forces:
                self.frc_left['x'], self.frc_left['y'], self.frc_left['z'] = data[3], data[4], data[5]
                self.frc_right['x'], self.frc_right['y'], self.frc_right['z'] = data[0], data[1], data[2]
            else:
                self.frc_left['x'] = 0
                self.frc_right['x'] = 0
                self.frc_left['y'], self.frc_left['z'] = data[2], data[3]
                self.frc_right['y'], self.frc_right['z'] = data[0], data[1]

            if self.dataExport[0]:
                t = round(time.time()-ti,2)
                f.write(str(t)+"\t"+"".join(map(lambda x: str(x)+'\t', data))+str("\n"))
                time.sleep(1/self.fs)
        print("Exiting ...")
        if self.dataExport[0]:
            f.close()

    def wait_for_exit(self):
        print("Press Enter for exit: ")
        x = stdin.readline()
        self.exitFlag = True

if __name__ == '__main__':
    frc = FrcAcquirer()
    thread1 = Thread(target = frc.wait_for_exit)
    thread1.start()
    frc.acquire()
