import os

# Isolate ROS 2 domain for unit tests to prevent port collision with live server
if "ROS_DOMAIN_ID" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "42"
