import time
import threading


stop_flag = threading.Event()
dt_list = []

def drummer(freq, signal_flag, stop_flag):
    # sets signal_flag to be set at regular intervals


    while not stop_flag.isSet():
        signal_flag.set()
        # dt_list.append(time.time() - start_time)
        time.sleep(1/freq - time.time() * freq % 1 / freq) # cuts off the fractional part of the amount of time, scaled.
    print "closing timer"


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    dt = 16 # milliseconds
    freq = 1000.0//16
    control_clock_flag = threading.Event()
    stop_flag = threading.Event()
    t = threading.Thread(target=drummer, args=(freq, control_clock_flag, stop_flag))

    t.start()

    counter = 0
    start_time = time.time()
    while counter < 1000:
        control_clock_flag.wait()
        dt_list.append(time.time() - start_time)
        control_clock_flag.clear()
        counter += 1

    stop_flag.set()

    dt_diff = []
    for i in xrange(len(dt_list)-1):
        dt_diff.append(dt_list[i+1]-dt_list[i])

    plt.plot(dt_diff[1:],'-o')
    plt.show()

