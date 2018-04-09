import time
import threading
import queue

stop_flag = threading.Event()
dt_list = []

def drummer(freq, signal_flag, stop_flag, q):
    # sets signal_flag to be set at regular intervals

    counter = 0
    while not stop_flag.isSet():
        signal_flag.set()
        # dt_list.append(time.time() - start_time)
        time.sleep(1/freq - time.time() * freq % 1 / freq) # cuts off the fractional part of the amount of time, scaled.
        q.put(counter)
        counter += 1

    print("closing timer")




if __name__ == '__main__':
    from matplotlib import pyplot as plt
    dt = 16 # milliseconds
    freq = 1000.0//dt
    q = queue.Queue()
    control_clock_flag = threading.Event()
    stop_flag = threading.Event()
    t = threading.Thread(target=drummer, args=(freq, control_clock_flag, stop_flag, q))

    t.start()
    q.put(-999)

    counter = 0
    start_time = time.time()
    while counter < 10000:
        control_clock_flag.wait()
        dt_list.append(time.time() - start_time)
        control_clock_flag.clear()
        counter += 1
        print(q.get())

    stop_flag.set()

    dt_diff = []
    for i in xrange(len(dt_list)-1):
        dt_diff.append(dt_list[i+1]-dt_list[i])

    plt.plot(dt_diff[1:],'-o')
    plt.show()

# open thread for Motion Manager.

# when you want to run a command, you place a command into the queue.

# queue listener is always listening. Every dt, it checks for the queue to have a command
    # if there is a command waiting, it runs the command and goes back to waiting.

