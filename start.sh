#!/bin/bash

if [ "$1" == "--help" ]; then
    echo "Usage: ./start.sh [BROKER_URL]"
    echo "Example: ./start.sh 192.168.218.123"
    exit 0
fi

BROKER_URL=$1

docker compose up --build -d
source .venv/bin/activate

if [ -z "$BROKER_URL" ]; then
    python3 MQTT_subscriber/mqtt_subscriber.py
else
    python3 MQTT_subscriber/mqtt_subscriber.py --broker_url $BROKER_URL
fi
