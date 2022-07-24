#!/bin/sh
mosquitto_pub -h localhost -t /pickup -m "{\"x\":11.316,\"y\":11.440,\"theta\":1.462}"
