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

def check_time(time_1, time_2):
    if time_1 > time_2:
        return time_1
    
    return time_2

if __name__ == "__main__":
    directory_path = "/mnt/c/Users/gonca/Desktop/received"
    file_name_1 = "F1_tests.csv"
    file_name_2 = "F2_tests.csv"
    file_name_3 = "F3_tests.csv"

    given_time = sys.argv[1]

    obj1 = time.strptime(given_time,'%Y.%m.%d-%H.%M.%S')
    start_time = time.mktime(obj1)

    #print(start_time + "\n")
    
    #start_time = time.time()
    print(f"Started looking at {time.ctime(start_time)}")

    creation_time_1 = file_creation_time(directory_path, file_name_1)
    creation_time_2 = file_creation_time(directory_path, file_name_2)
    #creation_time_3 = file_creation_time(directory_path, file_name_3)
    #end_time = time.time()

    final_time = check_time(creation_time_1, creation_time_2)
    
    print(f"File '{file_name_1}' was detected at {time.ctime(creation_time_1)}")
    print(f"File '{file_name_2}' was detected at {time.ctime(creation_time_2)}")

    print(f"Time taken: {final_time - start_time} seconds")
