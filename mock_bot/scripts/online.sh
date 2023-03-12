HOST="192.168.124.134"
# HOST="127.0.0.1"
PORT="8000"
ROUTER="/robot/online"

curl -X POST http://${HOST}:${PORT}${ROUTER} \
--header 'Content-Type: application/json' \
--data-raw '{"robot_uuid": "9d944006-bcc2-11ed-b9c9-bfe419b26b81","robot_username": "setsuna"}'
