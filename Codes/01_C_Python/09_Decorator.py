import os
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))

def processing_time(func):
    def w(*args, **kwargs):
        start_time = time.time()
        r = func(*args, **kwargs)
        end_time = time.time()
        print(f"[{func.__module__}]{func.__name__} exe time : {(end_time - start_time):.6f}")
        return r
    return w

@processing_time
def TxtFile_Read(str) :
    with open(str, 'r') as f:
        content = f.read()
    print(content)

if __name__ == '__main__' :    
    TxtFile_Read('filename.txt')