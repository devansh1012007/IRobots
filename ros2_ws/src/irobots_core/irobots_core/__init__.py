"""Pure-Python core for the irobots project.

PLAN:irobots-v2:D3 -- this package MUST NOT import ``rclpy`` or any other ROS
package. That separation is what makes the science testable without a ROS
graph, and it keeps ROS 2 deletable. CI enforces it.
"""

from __future__ import annotations

__version__ = "0.1.0"
