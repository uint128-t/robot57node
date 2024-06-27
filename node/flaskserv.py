import flask
import flask_socketio
import eventlet
import os
import os.path
import threading
from . import ros2serv

WEBSERVERPATH = os.path.dirname(os.path.realpath(__file__))+"/web"
print(f"Static folder at {WEBSERVERPATH}")

app = flask.Flask("Node 1")
socketio = flask_socketio.SocketIO(app,async_mode='eventlet')

@app.route("/")
def main():
    return flask.send_file(f"{WEBSERVERPATH}/index.html","text/html")
    # return flask.redirect("/static/index.html")

@app.route("/msg/<d>") # localhost only?
def message(d):
    socketio.emit(d,flask.request.args)
    return ""

@app.route("/web/<path:filename>")
def web(filename):
    return flask.send_from_directory(f"{WEBSERVERPATH}",filename)

@app.route("/ping")
def ping():
    return "online"

@socketio.on("goal")
def goal(d):
    print("RECEIVED GOAL")
    print(d)
    ros2serv.goals.put(d)
    # threading.Thread(target=goalsender.main,kwargs=d).start()

@socketio.on("vel")
def vel(d):
    ros2serv.vels.put(d)

def main(args=None):
    socketio.run(app,host="0.0.0.0",port=4809)
