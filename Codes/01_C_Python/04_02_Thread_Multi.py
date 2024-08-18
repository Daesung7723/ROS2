import threading
import time

def thread_1(stop_thread_1):
    print('Thread_1 Start')
    while True:
        print('Thread_1 working')
        time.sleep(0.5)
        if stop_thread_1.is_set() :
            print('Thread_1 is terminated')
            break

def thread_2(stop_thread_2):
    print('Thread_2 Start')
    while True:
        print('Thread_2 working')
        time.sleep(0.7)
        if stop_thread_2.is_set() :
            print('Thread_2 is terminated')
            break

def main():
    cnt=0
    while True:
        cnt = cnt+1
        print(f'Main code working : {cnt}')
        time.sleep(1)
        if cnt>10 :
            stop_thread_1.set()
            t1.join()
        if cnt>15 :
            stop_thread_2.set()
            t2.join()

if __name__ == '__main__' :    
    stop_thread_1 = threading.Event()
    stop_thread_2 = threading.Event()
    t1 =  threading.Thread(target=thread_1, args=(stop_thread_1,))
    t2 =  threading.Thread(target=thread_2, args=(stop_thread_2,))
    t1.start()
    t2.start()
    
    main()