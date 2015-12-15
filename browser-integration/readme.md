### Browser GUI for Kraken 3.0

- Install rosbridge

```sh
sudo apt-get install ros-indigo-rosbridge-suite
```

- Run the command to start the rosbridge server

```sh
roslaunch rosbridge_server rosbridge_websocket.launch
```

- Start a HTTP server in the folder that you have the HTML files in

```sh
python -m SimpleHTTPServer
```

- Visit the URL (generally `http://localhost:8000`) and append with the filename / filepath appropriately (something like `http://localhost:8000/simple.html` or `http://localhost:8000/graph_yaw_values.html`
