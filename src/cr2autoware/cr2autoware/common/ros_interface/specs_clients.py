"""
ROS Interface Client Specification
==================================

This file holds specification descriptions for all service clients of CR2Auto node.

To add a new service client, add the specification here first.

---------------------------
**Service Clients**:

* spec_change_to_stop_client:
    * Description: Service client for change to stop service call (only for publishing "stop" if goal arrived)
    * Topic: `/api/operation_mode/change_to_stop`
    * Message Type: `autoware_adapi_v1_msgs.srv.ChangeOperationMode`
"""

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.srv import ChangeOperationMode  # type: ignore

# cr2autoware imports
from .specs_base import SrvClientSpec

# =========================================


# client for change to stop service call (only for publishing "stop" if goal arrived)
spec_change_to_stop_client = SrvClientSpec(name="/api/operation_mode/change_to_stop",
                                           srv_type=ChangeOperationMode)
