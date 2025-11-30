import rclpy
from rclpy.node import Node
from s1_interfaces.msg import Telemetry, LogEntry, HealthReport

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__("diagnostics_node")

        # Subscribe to telemetry
        self.sub = self.create_subscription(
            Telemetry,
            "/s1/telemetry/raw",
            self.cb_telemetry,
            10
        )

        # Publishers
        self.pub_report = self.create_publisher(HealthReport, "/s1/diagnostics/report", 10)
        self.pub_log = self.create_publisher(LogEntry, "/s1/logs/raw", 10)

        # Cached telemetry
        self.last = None

        # Timer to evaluate at 1 Hz
        self.timer = self.create_timer(1.0, self.evaluate)
        
        self.get_logger().info("Diagnostics node started - monitoring system health")

    def cb_telemetry(self, msg):
        self.last = msg

    def make_log(self, level, message):
        log = LogEntry()
        log.stamp = self.get_clock().now().to_msg()
        log.level = level
        log.source_node = "diagnostics_node"
        log.message = message
        self.pub_log.publish(log)
        self.get_logger().info(f"[DIAG LOG] {level}: {message}")

    def evaluate(self):
        if self.last is None:
            return

        hr = HealthReport()
        hr.stamp = self.get_clock().now().to_msg()
        hr.source_node = "diagnostics_node"

        # Default state
        hr.status = "OK"
        hr.level = 0  # OK
        issues = []

        # --------------------------
        # POWER CHECKS
        # --------------------------
        if self.last.battery_pct < 20:
            hr.status = "WARN"
            hr.level = max(hr.level, 1)
            issues.append(f"Battery low: {self.last.battery_pct:.1f}%")

        if self.last.voltage_v < 22.0:
            hr.status = "WARN"
            hr.level = max(hr.level, 1)
            issues.append(f"Voltage low: {self.last.voltage_v:.1f}V")

        # --------------------------
        # THERMAL
        # --------------------------
        if self.last.temperature_c > 60:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"Temperature HIGH: {self.last.temperature_c:.1f}°C")

        # --------------------------
        # INTERFACES DOWN
        # --------------------------
        if self.last.p2_status == "DOWN":
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append("P2 interface DOWN")
            
        if self.last.p3_status == "DOWN":
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append("P3 interface DOWN")
            
        if self.last.p4_status == "DOWN":
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append("P4 interface DOWN")

        # --------------------------
        # DISK SPACE
        # --------------------------
        if self.last.p2_disk_free < 20:
            if hr.level < 2:
                hr.status = "WARN"
                hr.level = max(hr.level, 1)
            issues.append(f"P2 disk low: {self.last.p2_disk_free:.1f}%")
            
        if self.last.p3_disk_free < 20:
            if hr.level < 2:
                hr.status = "WARN"
                hr.level = max(hr.level, 1)
            issues.append(f"P3 disk low: {self.last.p3_disk_free:.1f}%")
            
        if self.last.p4_disk_free < 20:
            if hr.level < 2:
                hr.status = "WARN"
                hr.level = max(hr.level, 1)
            issues.append(f"P4 disk low: {self.last.p4_disk_free:.1f}%")

        # --------------------------
        # SYSTEM LOAD
        # --------------------------
        if self.last.p2_cpu_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P2 CPU critical: {self.last.p2_cpu_percent:.1f}%")
            
        if self.last.p3_cpu_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P3 CPU critical: {self.last.p3_cpu_percent:.1f}%")
            
        if self.last.p4_cpu_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P4 CPU critical: {self.last.p4_cpu_percent:.1f}%")

        if self.last.p2_mem_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P2 Memory critical: {self.last.p2_mem_percent:.1f}%")
            
        if self.last.p3_mem_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P3 Memory critical: {self.last.p3_mem_percent:.1f}%")
            
        if self.last.p4_mem_percent > 90:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append(f"P4 Memory critical: {self.last.p4_mem_percent:.1f}%")

        # --------------------------
        # ESTOP
        # --------------------------
        if self.last.estop:
            hr.status = "CRITICAL"
            hr.level = 2
            issues.append("⚠️ EMERGENCY STOP ACTIVE")

        # --------------------------
        # Build HealthReport
        # --------------------------
        hr.issues = issues
        hr.message = ", ".join(issues) if issues else "All systems nominal"

        # Publish health report
        self.pub_report.publish(hr)

        # Emit log for significant issues
        if hr.level == 2:  # CRITICAL
            self.make_log("ERROR", hr.message)
        elif hr.level == 1:  # WARN
            self.make_log("WARN", hr.message)
        elif len(issues) == 0:  # Only log OK periodically
            if hasattr(self, '_ok_counter'):
                self._ok_counter += 1
                if self._ok_counter % 10 == 0:  # Every 10 seconds
                    self.make_log("INFO", "System health check: All systems nominal")
            else:
                self._ok_counter = 0

        self.get_logger().info(
            f"Health: {hr.status} | Issues: {len(issues)} | "
            f"Batt: {self.last.battery_pct:.1f}% | "
            f"Temp: {self.last.temperature_c:.1f}°C"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
