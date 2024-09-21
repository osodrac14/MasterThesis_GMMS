from datetime import datetime
import os
import subprocess
import threading

move_dir_recv = f'HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/receiver/'
recv_file = f'./start_ltp_receive_bpv6.sh'

dtn_send = f'cp /home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/receiver/data.csv /home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata'
move_dir_send = f'HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/'
send_file = f'./start_ltp_send_bpv6.sh'

def verify_internet_connection():
    try:
        subprocess.check_output(["ping", "-c", "1", "192.168.1.76"])
        return True
    except subprocess.CalledProcessError:
        return False

def transfer_dtn_file():

    print("Sending file over DTN")
    os.chdir(move_dir_send)
    dispatch = subprocess.run(send_file, shell=True)
    print("DTN running...")
    if dispatch.returncode == 0:
        print("File transfer successful!")

    else:
        print(f"DTN transfer failed with return code {dispatch.returncode}")
    
    return dispatch.returncode

def send_recved_thread():
    
    os.environ['HDTN_SOURCE_ROOT'] = '/home/pi/HDTN'
    print(f"Defined source_root: {os.getenv('HDTN_SOURCE_ROOT')}")

    while True:
        # Check internet connection with receiving node
        if verify_internet_connection():
            subprocess.run(dtn_send, shell=True)
            if os.path.exists('/home/pi/HDTN/tests/test_scripts_linux/LCRD_File_Transfer_Test/sender/flightdata/data.csv'):
                dtn_transfer_start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print(f"Internet connection available. Initiating DTN transfer at {dtn_transfer_start_time}.")
                transfer_success = transfer_dtn_file()

                if transfer_success == 0:
                    print("Transfer completed!")
                    return
                else:
                    print(f"DTN transfer failed with return code {transfer_success}")
            else:
                print("The desired file doesn't exist.")

        else:
            print("No internet connection available.")
    

if __name__ == "__main__":

    print("Starting...")

    os.environ['HDTN_SOURCE_ROOT'] = '/home/pi/HDTN'

    transmission_thread = threading.Thread(target=send_recved_thread)
    transmission_thread.start()

    os.chdir(move_dir_recv)
    dispatch = subprocess.run(recv_file, shell=True)

    transmission_thread.join()