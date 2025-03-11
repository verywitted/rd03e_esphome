#!/bin/bash

# RD03-E Radar MQTT Project Initialization Script
# This script helps set up the project for first-time use

echo "Initializing RD03-E Radar MQTT Project..."

# Create secrets.h from example if it doesn't exist
if [ ! -f include/secrets.h ]; then
  echo "Creating secrets.h from example..."
  cp include/secrets.h.example include/secrets.h
  echo "Please edit include/secrets.h to add your WiFi and MQTT credentials"
else
  echo "secrets.h already exists, keeping existing file"
fi

# Ensure all directories exist
mkdir -p .vscode 2>/dev/null

# Create VSCode configuration if it doesn't exist
if [ ! -f .vscode/extensions.json ]; then
  echo "Creating VSCode extensions recommendations..."
  cat > .vscode/extensions.json << EOL
{
  "recommendations": [
    "platformio.platformio-ide",
    "ms-vscode.cpptools"
  ]
}
EOL
fi

# Initialize PlatformIO if platformio.ini exists but the .pio directory doesn't
if [ -f platformio.ini ] && [ ! -d .pio ]; then
  echo "Initializing PlatformIO project..."
  pio project init --ide vscode
else
  echo "PlatformIO project already initialized or platformio.ini missing"
fi

echo "Done! You can now build the project with: pio run"
echo "Don't forget to edit include/secrets.h with your credentials"