import time
import serial
import random
import numpy as np
import scipy.signal
import matplotlib.pyplot as plt
import matplotlib.animation as animation


PI = np.pi
g = 9.80665
port_name = "COM3"
baudrate = 115200

def period(om):
    tic = 0.01
    T = 0.
    T1 = []
    T2 = []
    m = np.mean(om)
    f = 1
    k1 = 0
    k2 = 0
    l_om = len(om)
    time1 = []
    time2 = []
    i = 0
    if om[0] < m:
        while i < l_om:
            if om[i] > m and f == 1:
                time1.append(i)
                f = 0
                continue
            if om[i] < m and f == 0:
                time2.append(i)
                f = 1
                continue
            i += 1
    else:
        while i < l_om:
            if om[i] < m and f == 1:
                time1.append(i)
                f = 0
                continue
            if om[i] > m and f == 0:
                time2.append(i)
                f = 1
                continue
            i += 1
    k1 = len(time1)
    k2 = len(time2)
    k = k1 if k1 <= k2 else k2   
    for i in range(1, k):
        T1.append((time1[i] - time1[i-1]) * tic)        
        T2.append( (time2[i] - time2[i-1]) * tic)
    T = (np.mean(T1) + np.mean(T2)) / 2
    if T > 1.5:
        T += 0.075 * T
    else:
        T += 0.05 * T
    return T, time1, time2
    
    
def length(T, g, PI):
    l = 0.
    l = (T * T * g) / (4 * PI * PI)
    return l  

def Euler(dataList):
    h = 0.01 #tic
    t = np.arange(0, 7 + h, h)
    s0 = -1
    s = np.zeros(len(dataList))
    s[0] = s0
    
    for i in range(0, len(t) - 2):
        s[i + 1] = s[i] + h * dataList[i]
    return s
    
def animate(i, ox, oy, oz, dataList, ser):    
    try:        
        stm32_string = ser.readline().decode("utf-8")  
        stm32_string = stm32_string.strip().split()           
        stm32_float = np.float64(stm32_string)
        ox_n = stm32_float[0::3]
        oy_n = stm32_float[1::3]
        oz_n = stm32_float[2::3]
        
        for i in range(1, len(ox_n)):
            if ox_n[i] > 250000 or ox_n[i] < -250000:
                ox_n[i] = ox_n[i - 1]
        for i in range(1, len(oy_n)):
            if oy_n[i] > 250000 or oy_n[i] < -250000:
                oy_n[i] = oy_n[i - 1]
        for i in range(1, len(oz_n)):
            if oz_n[i] > 250000 or oz_n[i] < -250000:
                oz_n[i] = oz_n[i - 1]               
        
        ox.extend(ox_n)
        oy.extend(oy_n)
        oz.extend(oz_n)         
                
        ox = ox[-700:]
        oy = oy[-700:]
        oz = oz[-700:]
        om = (np.float64(ox)**2 + np.float64(oy)**2 + np.float64(oz)**2)**0.5
        
        s1 = s2 = s3 = 0        
        for i in range(700):
            if ox[i] > 0:
                s1 += ox[i]
        for i in range(700):
            if oy[i] > 0:
                s2 += oy[i]
        for i in range(700):
            if oz[i] > 0:
                s3 += oz[i]
        if s1 > s2 and s1 > s3:
            s = np.float64(ox) +  np.float64(om)
        elif s2 > s1 and s2 > s3:
            s = np.float64(oy) + np.float64(om)
        else:
            s = np.float64(oz) + np.float64(om)
        s = scipy.signal.medfilt(s, kernel_size=9)
        
        dataList[-700:] = s
    except Exception as error:
        print("An exception occured: ", error)
        return
    
    m = np.mean(dataList)    
    theta_z = Euler(oz)
    time1 = []
    time2 = []
    try:
        T, time1, time2 = period(dataList)
        T_str = str(T)
        l = length(T, g, PI)
        l_str = str(l)
    except ZeroDivisionError:
        T = 0
        T_str = str(T)
        l = 0
        l_str = str(l)
    
    ####################################################################################
    string = '   Длина l = ' + l_str
    axs[0, 0].clear()
    axs[0, 0].text(0.2, 0.8, string)
    string = 'Период T = ' + T_str
    axs[0, 0].text(0.2, 0.7, string)
    ####################################################################################
    axs[0, 1].clear()
    axs[0, 1].plot(ox, label="$\omega_x$")
    axs[0, 1].plot(oy, label="$\omega_y$")
    axs[0, 1].plot(oz, label="$\omega_z$")     
    axs[0, 1].set_ylim([-300000, 300000])
    axs[0, 1].set_xlabel("Последние 700 отсчетов")   
    axs[0, 1].grid()
    axs[0, 1].legend(loc="upper left")
    ####################################################################################    
    axs[1, 0].clear() 
    axs[1, 0].plot(dataList, label="$\max(\omega_x, \omega_y, \omega_z)+\omega$") 
    axs[1, 0].plot(range(700), m*np.ones(700), label="mean")    
    axs[1, 0].scatter(time1, m * np.ones(len(time1)), marker="x", s=150, c='red')
    axs[1, 0].scatter(time2, m * np.ones(len(time2)), marker="x", s=150, c='green')
    axs[1, 0].set_ylim([-150000, 350000])
    axs[1, 0].set_xlabel("Последние 700 отсчетов")    
    axs[1, 0].grid()
    axs[1, 0].legend(loc="upper left") 
    ####################################################################################
    axs[1, 1].clear()
    axs[1, 1].plot(theta_z, oz)
    axs[1, 1].set_xlabel("$z$")  
    axs[1, 1].set_ylabel("$\omega_z$")    
    axs[1, 1].grid()
    ####################################################################################


dataList = 700 * [0]
ox       = 700 * [0]
oy       = 700 * [0]
oz       = 700 * [0]

fig, axs = plt.subplots(2, 2)
fig.set_size_inches(13, 6)
fig.suptitle('Маятник', fontsize=16)

ser = serial.Serial(port_name, baudrate)  
ani = animation.FuncAnimation(fig, animate, frames=None, fargs=(ox, oy, oz, dataList, ser))   

plt.show()
ser.close()   



