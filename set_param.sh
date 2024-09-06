#!/bin/bash
#Example for setting parameters like bitrate over JSON RPC. Sets main transmit stream bitrate to 2000 kbps and full stabilization.
curl -X POST -H 'Content-Type: application/json' -d '{"jsonrpc":"2.0","method":"setParam","params":{"args":{"bitrate0": 2000, "stab": 4}}}' http://localhost:5000
