#!/bin/bash
#Example for setting parameters like bitrate over JSON RPC. Sets main transmit stream bitrate to 2000 kbps and full stabilization.
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"setParam","params":{"args":{"stream0_bitrate": 2000, "stab": 3}}}' http://localhost:5000

#Example for setting parameters like bitrate over JSON RPC. Sets main transmit stream bitrate to 15000 kbps and x/y stabilization
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"setParam","params":{"args":{"stream0_bitrate": 15000, "stab": 1}}}' http://localhost:5000

#Switches udpsink 0 stream to the secondary stream
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"setParam","params":{"args":{"output0_stream": 1}}}' http://localhost:5000

# set the clients of udpsink 1 to two ips and mtu to 512 bytes
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"setParam","params":{"args":{"clients1": "127.0.0.1:6000,192.168.1.100:5600", "stream1_mtu": 512}}}' http://localhost:5000

#get current parameters
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"getParam","params":{}, "id":0}' http://localhost:5000



