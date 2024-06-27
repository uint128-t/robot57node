from . import flaskserv, ros2serv
import threading

def main(args=None):
    p=threading.Thread(target=ros2serv.main,daemon=True)
    p.start()
    flaskserv.main()
    print("Finished node1")

if __name__ == "__main__":
    main()