#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

"""
Watchdog facility for monitoring Composer subsystem health.
Periodically pings Composer services and reports which are up/down.
"""

import json
import time
from typing import Dict, Any, Optional, List
from enum import IntEnum
from dataclasses import dataclass, field
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String


class HealthStatus(IntEnum):
    """Health status levels for subsystems."""
    HEALTHY = 0
    DEGRADED = 1
    FAILED = 2
    UNKNOWN = 3


@dataclass
class SubsystemHealth:
    """Health status for a single subsystem."""
    name: str
    status: HealthStatus = HealthStatus.UNKNOWN
    message: str = ""
    last_check: Optional[float] = None
    response_time_ms: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status.name,
            "message": self.message,
            "last_check": self.last_check,
            "response_time_ms": self.response_time_ms,
        }


@dataclass
class SystemHealthReport:
    """Overall system health report."""
    overall_status: HealthStatus = HealthStatus.UNKNOWN
    subsystems: Dict[str, SubsystemHealth] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    uptime_seconds: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "overall_status": self.overall_status.name,
            "subsystems": {name: sub.to_dict() for name, sub in self.subsystems.items()},
            "timestamp": self.timestamp,
            "uptime_seconds": self.uptime_seconds,
        }


class ComposerWatchdog(Node):
    """
    Watchdog facility for monitoring Composer subsystems.

    Periodically checks the health of:
    - MutoDefaultLaunchPlugin (muto_start_stack service)
    - MutoProvisionPlugin (muto_provision service)
    - MutoDefaultComposePlugin (muto_compose service)
    - CoreTwin service (core_twin services)

    Publishes health status to a ROS topic and provides a service
    for on-demand health queries.
    """

    SERVICES_TO_MONITOR = [
        ("launch_plugin", "muto_start_stack"),
        ("provision_plugin", "muto_provision"),
        ("compose_plugin", "muto_compose"),
        ("core_twin", "core_twin/get_stack_definition"),
    ]

    def __init__(
        self,
        node_name: str = "composer_watchdog",
        check_interval_sec: float = 10.0,
        service_timeout_sec: float = 2.0,
    ):
        super().__init__(node_name)

        self.check_interval_sec = check_interval_sec
        self.service_timeout_sec = service_timeout_sec
        self.start_time = time.time()

        self._callback_group = ReentrantCallbackGroup()

        # Initialize health tracking
        self._subsystem_health: Dict[str, SubsystemHealth] = {}
        for name, _ in self.SERVICES_TO_MONITOR:
            self._subsystem_health[name] = SubsystemHealth(name=name)

        # Create health status publisher
        self._health_pub = self.create_publisher(
            String,
            "composer_watchdog/health_status",
            10
        )

        # Create health check service
        self._health_service = self.create_service(
            Trigger,
            "composer_watchdog/check_health",
            self._handle_health_check,
            callback_group=self._callback_group
        )

        # Create service clients for each monitored service
        self._service_clients: Dict[str, Any] = {}
        self._init_service_clients()

        # Create periodic health check timer
        self._check_timer = self.create_timer(
            self.check_interval_sec,
            self._periodic_health_check,
            callback_group=self._callback_group
        )

        self.get_logger().info(
            f"ComposerWatchdog initialized. Checking {len(self.SERVICES_TO_MONITOR)} "
            f"services every {check_interval_sec}s"
        )

    def _init_service_clients(self) -> None:
        """Initialize ROS2 service clients for health probing."""
        # We only need to check service availability, not call them
        # So we just track which services exist
        pass

    def _check_service_availability(self, service_name: str) -> bool:
        """Check if a ROS2 service is available."""
        service_names_and_types = self.get_service_names_and_types()
        full_service_name = f"/{service_name}" if not service_name.startswith("/") else service_name

        for svc_name, _ in service_names_and_types:
            if svc_name == full_service_name:
                return True
        return False

    def _perform_health_check(self) -> SystemHealthReport:
        """Perform health check on all monitored subsystems."""
        report = SystemHealthReport(
            timestamp=time.time(),
            uptime_seconds=time.time() - self.start_time
        )

        healthy_count = 0
        degraded_count = 0
        failed_count = 0

        for subsystem_name, service_name in self.SERVICES_TO_MONITOR:
            start = time.time()
            health = SubsystemHealth(name=subsystem_name)

            try:
                is_available = self._check_service_availability(service_name)
                response_time = (time.time() - start) * 1000

                if is_available:
                    health.status = HealthStatus.HEALTHY
                    health.message = f"Service {service_name} is available"
                    healthy_count += 1
                else:
                    health.status = HealthStatus.FAILED
                    health.message = f"Service {service_name} not found"
                    failed_count += 1

                health.response_time_ms = response_time

            except Exception as e:
                health.status = HealthStatus.FAILED
                health.message = f"Error checking {service_name}: {str(e)}"
                failed_count += 1

            health.last_check = time.time()
            self._subsystem_health[subsystem_name] = health
            report.subsystems[subsystem_name] = health

        # Determine overall health
        if failed_count > 0:
            if healthy_count > 0:
                report.overall_status = HealthStatus.DEGRADED
            else:
                report.overall_status = HealthStatus.FAILED
        elif degraded_count > 0:
            report.overall_status = HealthStatus.DEGRADED
        else:
            report.overall_status = HealthStatus.HEALTHY

        return report

    def _periodic_health_check(self) -> None:
        """Timer callback to perform periodic health checks and publish results."""
        try:
            report = self._perform_health_check()

            # Publish health status
            msg = String()
            msg.data = json.dumps(report.to_dict())
            self._health_pub.publish(msg)

            # Log summary
            status_summary = ", ".join(
                f"{name}={sub.status.name}"
                for name, sub in report.subsystems.items()
            )
            self.get_logger().info(
                f"Health check complete: {report.overall_status.name} [{status_summary}]"
            )

        except Exception as e:
            self.get_logger().error(f"Error during health check: {e}")

    def _handle_health_check(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Handle on-demand health check service requests."""
        try:
            report = self._perform_health_check()

            response.success = report.overall_status != HealthStatus.FAILED
            response.message = json.dumps(report.to_dict())

        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {str(e)}"

        return response

    def get_health_report(self) -> SystemHealthReport:
        """Get current health report (for programmatic access)."""
        return self._perform_health_check()

    def get_subsystem_health(self, name: str) -> Optional[SubsystemHealth]:
        """Get health status for a specific subsystem."""
        return self._subsystem_health.get(name)


def main(args=None):
    """Main entry point for the Composer Watchdog node."""
    rclpy.init(args=args)
    watchdog = ComposerWatchdog()
    try:
        rclpy.spin(watchdog)
    except KeyboardInterrupt:
        pass
    finally:
        watchdog.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
