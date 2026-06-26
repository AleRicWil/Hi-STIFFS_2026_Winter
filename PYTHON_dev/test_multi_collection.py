from config import Config
from collect_data import run_collection
import threading

Config.start_new_data_session()   # Locks the exact same timestamp for all CSVs

def launch(nid, sens, sns):
    run_collection(sensors=sens, sensor_sns=sns, nano_id=nid, plot=False)

t1 = threading.Thread(target=launch, args=(1, "A B C D E", "001,002,003,004,005"))
t2 = threading.Thread(target=launch, args=(2, "A B", "101,102"))
t1.start(); t2.start()
t1.join(); t2.join()