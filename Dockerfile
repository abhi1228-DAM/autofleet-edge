FROM python:3.12-slim

WORKDIR /app

COPY autofleet_go_to.py ros_action_bridge.py README.md ./

CMD ["python3", "autofleet_go_to.py", "--help"]
