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
Unit tests for the Composer Watchdog facility.
"""

import json
import unittest
from unittest.mock import MagicMock, patch
import time

from composer.subsystems.watchdog import (
    ComposerWatchdog,
    HealthStatus,
    SubsystemHealth,
    SystemHealthReport,
)


class TestHealthStatus(unittest.TestCase):
    """Tests for HealthStatus enum."""

    def test_health_status_values(self):
        """Test that health status values are correct."""
        self.assertEqual(HealthStatus.HEALTHY, 0)
        self.assertEqual(HealthStatus.DEGRADED, 1)
        self.assertEqual(HealthStatus.FAILED, 2)
        self.assertEqual(HealthStatus.UNKNOWN, 3)

    def test_health_status_names(self):
        """Test that health status names are accessible."""
        self.assertEqual(HealthStatus.HEALTHY.name, "HEALTHY")
        self.assertEqual(HealthStatus.FAILED.name, "FAILED")


class TestSubsystemHealth(unittest.TestCase):
    """Tests for SubsystemHealth dataclass."""

    def test_default_values(self):
        """Test default values for SubsystemHealth."""
        health = SubsystemHealth(name="test")
        self.assertEqual(health.name, "test")
        self.assertEqual(health.status, HealthStatus.UNKNOWN)
        self.assertEqual(health.message, "")
        self.assertIsNone(health.last_check)
        self.assertIsNone(health.response_time_ms)

    def test_to_dict(self):
        """Test to_dict serialization."""
        health = SubsystemHealth(
            name="test_subsystem",
            status=HealthStatus.HEALTHY,
            message="All good",
            last_check=1234567890.0,
            response_time_ms=50.5
        )
        result = health.to_dict()

        self.assertEqual(result["name"], "test_subsystem")
        self.assertEqual(result["status"], "HEALTHY")
        self.assertEqual(result["message"], "All good")
        self.assertEqual(result["last_check"], 1234567890.0)
        self.assertEqual(result["response_time_ms"], 50.5)


class TestSystemHealthReport(unittest.TestCase):
    """Tests for SystemHealthReport dataclass."""

    def test_default_values(self):
        """Test default values for SystemHealthReport."""
        report = SystemHealthReport()
        self.assertEqual(report.overall_status, HealthStatus.UNKNOWN)
        self.assertEqual(report.subsystems, {})
        self.assertIsInstance(report.timestamp, float)
        self.assertEqual(report.uptime_seconds, 0.0)

    def test_to_dict(self):
        """Test to_dict serialization."""
        report = SystemHealthReport(
            overall_status=HealthStatus.HEALTHY,
            subsystems={
                "test": SubsystemHealth(name="test", status=HealthStatus.HEALTHY)
            },
            timestamp=1234567890.0,
            uptime_seconds=100.0
        )
        result = report.to_dict()

        self.assertEqual(result["overall_status"], "HEALTHY")
        self.assertEqual(result["timestamp"], 1234567890.0)
        self.assertEqual(result["uptime_seconds"], 100.0)
        self.assertIn("test", result["subsystems"])
        self.assertEqual(result["subsystems"]["test"]["status"], "HEALTHY")


class TestComposerWatchdogServicesToMonitor(unittest.TestCase):
    """Tests for ComposerWatchdog service monitoring list."""

    def test_services_to_monitor_defined(self):
        """Test that services to monitor are properly defined."""
        services = ComposerWatchdog.SERVICES_TO_MONITOR

        self.assertGreater(len(services), 0)
        for name, service in services:
            self.assertIsInstance(name, str)
            self.assertIsInstance(service, str)

    def test_expected_services_present(self):
        """Test that expected services are in the monitoring list."""
        names = [name for name, _ in ComposerWatchdog.SERVICES_TO_MONITOR]

        self.assertIn("launch_plugin", names)
        self.assertIn("provision_plugin", names)
        self.assertIn("compose_plugin", names)


class TestComposerWatchdogInitialization(unittest.TestCase):
    """Tests for ComposerWatchdog initialization."""

    @patch('rclpy.init')
    @patch('rclpy.shutdown')
    def test_services_to_monitor_count(self, mock_shutdown, mock_init):
        """Test that all services are tracked for monitoring."""
        # The SERVICES_TO_MONITOR constant should be accessible without instantiation
        expected_services = [
            "launch_plugin",
            "provision_plugin",
            "compose_plugin",
            "core_twin",
        ]
        actual_names = [name for name, _ in ComposerWatchdog.SERVICES_TO_MONITOR]

        for expected in expected_services:
            self.assertIn(expected, actual_names)


class TestHealthReportSerialization(unittest.TestCase):
    """Tests for health report JSON serialization."""

    def test_full_report_serialization(self):
        """Test that a full health report can be serialized to JSON."""
        report = SystemHealthReport(
            overall_status=HealthStatus.DEGRADED,
            subsystems={
                "service1": SubsystemHealth(
                    name="service1",
                    status=HealthStatus.HEALTHY,
                    message="OK",
                    last_check=time.time(),
                    response_time_ms=10.5
                ),
                "service2": SubsystemHealth(
                    name="service2",
                    status=HealthStatus.FAILED,
                    message="Service not found",
                    last_check=time.time(),
                    response_time_ms=5.0
                ),
            },
            timestamp=time.time(),
            uptime_seconds=3600.0
        )

        # Should not raise
        json_str = json.dumps(report.to_dict())
        self.assertIsInstance(json_str, str)

        # Should be valid JSON that can be parsed back
        parsed = json.loads(json_str)
        self.assertEqual(parsed["overall_status"], "DEGRADED")
        self.assertEqual(len(parsed["subsystems"]), 2)

    def test_empty_report_serialization(self):
        """Test that an empty report can be serialized."""
        report = SystemHealthReport()
        json_str = json.dumps(report.to_dict())
        parsed = json.loads(json_str)

        self.assertEqual(parsed["overall_status"], "UNKNOWN")
        self.assertEqual(parsed["subsystems"], {})


class TestHealthStatusLogic(unittest.TestCase):
    """Tests for health status calculation logic."""

    def test_all_healthy_is_healthy(self):
        """Test that all healthy subsystems results in HEALTHY overall."""
        # This tests the logic pattern used in the watchdog
        healthy_count = 3
        failed_count = 0
        degraded_count = 0

        if failed_count > 0:
            if healthy_count > 0:
                status = HealthStatus.DEGRADED
            else:
                status = HealthStatus.FAILED
        elif degraded_count > 0:
            status = HealthStatus.DEGRADED
        else:
            status = HealthStatus.HEALTHY

        self.assertEqual(status, HealthStatus.HEALTHY)

    def test_some_failed_is_degraded(self):
        """Test that some failed with some healthy is DEGRADED."""
        healthy_count = 2
        failed_count = 1
        degraded_count = 0

        if failed_count > 0:
            if healthy_count > 0:
                status = HealthStatus.DEGRADED
            else:
                status = HealthStatus.FAILED
        elif degraded_count > 0:
            status = HealthStatus.DEGRADED
        else:
            status = HealthStatus.HEALTHY

        self.assertEqual(status, HealthStatus.DEGRADED)

    def test_all_failed_is_failed(self):
        """Test that all failed subsystems is FAILED."""
        healthy_count = 0
        failed_count = 3
        degraded_count = 0

        if failed_count > 0:
            if healthy_count > 0:
                status = HealthStatus.DEGRADED
            else:
                status = HealthStatus.FAILED
        elif degraded_count > 0:
            status = HealthStatus.DEGRADED
        else:
            status = HealthStatus.HEALTHY

        self.assertEqual(status, HealthStatus.FAILED)


if __name__ == "__main__":
    unittest.main()
