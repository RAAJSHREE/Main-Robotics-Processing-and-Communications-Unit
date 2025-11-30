#!/usr/bin/env powershell
# Complete System Startup Script for Windows

Write-Host "🤖 Main Robotics Processing and Communications Unit" -ForegroundColor Cyan
Write-Host "=================================================" -ForegroundColor Cyan
Write-Host ""

$rootPath = "C:\Project\Main-Robotics-Processing-and-Communications-Unit\Main-Robotics-Processing-and-Communications-Unit"

# Step 1: Start Docker Services
Write-Host "📦 Step 1: Starting Docker services..." -ForegroundColor Yellow
Set-Location "$rootPath\Dashboard"

docker-compose up -d

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ Docker services started" -ForegroundColor Green
} else {
    Write-Host "❌ Failed to start Docker services" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "⏳ Waiting for MQTT broker to be ready..." -ForegroundColor Yellow
Start-Sleep -Seconds 5

# Step 2: Check ROS2 workspace exists
Write-Host ""
Write-Host "🔧 Step 2: Preparing ROS2 workspace..." -ForegroundColor Yellow

$ros2LaunchScript = "$rootPath/scripts/launch_full_system.sh"

if (Test-Path $ros2LaunchScript) {
    Write-Host "✅ ROS2 launch script found" -ForegroundColor Green
} else {
    Write-Host "❌ ROS2 launch script not found at: $ros2LaunchScript" -ForegroundColor Red
    exit 1
}

# Step 3: Launch ROS2 in new terminal
Write-Host ""
Write-Host "🚀 Step 3: Launching ROS2 nodes (13 nodes)..." -ForegroundColor Yellow
Write-Host "   Opening new terminal window for ROS2..." -ForegroundColor Gray

Start-Process powershell -ArgumentList @(
    "-NoExit",
    "-Command",
    "wsl bash /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/scripts/launch_full_system.sh"
)

Write-Host "✅ ROS2 launch initiated" -ForegroundColor Green

Start-Sleep -Seconds 3

# Step 4: Open Dashboard
Write-Host ""
Write-Host "🌐 Step 4: Opening dashboard..." -ForegroundColor Yellow
Start-Sleep -Seconds 2
Start-Process "http://localhost:8080"
Write-Host "✅ Dashboard opened in browser" -ForegroundColor Green

# Summary
Write-Host ""
Write-Host "=================================================" -ForegroundColor Cyan
Write-Host "✅ SYSTEM STARTUP COMPLETE" -ForegroundColor Green
Write-Host "=================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "📊 Dashboard:    http://localhost:8080" -ForegroundColor White
Write-Host "🔧 Backend API:  http://localhost:8000" -ForegroundColor White
Write-Host "📡 MQTT Broker:  localhost:1883" -ForegroundColor White
Write-Host ""
Write-Host "🔍 To verify ROS2 nodes are running:" -ForegroundColor Yellow
Write-Host "   wsl bash -c 'source /opt/ros/humble/setup.bash && source /mnt/c/Project/Main-Robotics-Processing-and-Communications-Unit/Main-Robotics-Processing-and-Communications-Unit/s1_ws/install/setup.bash && ros2 node list'" -ForegroundColor Gray
Write-Host ""
Write-Host "🛑 To stop services:" -ForegroundColor Yellow
Write-Host "   1. Press Ctrl+C in ROS2 terminal" -ForegroundColor Gray
Write-Host "   2. Run: cd Dashboard; docker-compose down" -ForegroundColor Gray
Write-Host ""
