import threading
import time

def thread_1():
    while True:
        print('Thread_1 working')
        time.sleep(0.5)


if __name__ == '__main__' :

    t1 =  threading.Thread(target=thread_1)
    # t1 =  threading.Thread(target=thread_1, daemon=True)
    t1.start()

    while True:
        print('Main code working')
        time.sleep(2)
