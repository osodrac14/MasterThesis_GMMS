import os
import time
import sys

def file_creation_time(directory, filename):
    filepath = os.path.join(directory, filename)
    
    # Check if the file already exists
    if os.path.exists(filepath):
        return os.path.getctime(filepath)
    
    # Monitor the directory for changes
    while True:
        if os.path.exists(filepath):
            return os.path.getctime(filepath)
        time.sleep(1)

if __name__ == "__main__":
    directory_path = "/mnt/c/Users/gonca/Desktop/Tese"
    file_name = "F1_tests.csv"

    given_time = sys.argv[1]

    obj1 = time.strptime(given_time,'%Y.%m.%d-%H.%M.%S')
    start_time = time.mktime(obj1)

    #print(start_time + "\n")
    
    #start_time = time.time()
    print(f"Started looking at {time.ctime(start_time)}")

    creation_time = file_creation_time(directory_path, file_name)
    #end_time = time.time()
    
    print(f"File '{file_name}' was detected at {time.ctime(creation_time)}")
    print(f"Time taken: {creation_time - start_time} seconds")
