#!/bin/bash

# Define variables for future use
SERVICE_FILE_NAME_LCD="LCD.service"
SERVICE_FILE_PATH_LCD="../mission/LCD/startup_scripts/"
SERVICE_FILE_NAME_INTERNAL_STATUS_AUV="internal_status_auv.service"
SERVICE_FILE_PATH_INTERNAL_STATUS_AUV="../mission/internal_status_auv/startup_scripts/"
SERVICE_FILE_NAME_BLACKBOX="blackbox.service"
SERVICE_FILE_PATH_BLACKBOX="../mission/blackbox/startup_scripts/"

SYSTEMD_PATH="/etc/systemd/system/"
PYTHON_VERSION="python3.10"



# Navigate ----------
echo "Navigating to correct directory..."
# Get scripts directory and go there
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
cd $SCRIPT_DIR



# Setup ----------
echo "Setting up .service files..."
# Copy the .service files to current directory
cp $SERVICE_FILE_PATH_LCD$SERVICE_FILE_NAME_LCD $SERVICE_FILE_NAME_LCD # LCD
cp $SERVICE_FILE_PATH_INTERNAL_STATUS_AUV$SERVICE_FILE_NAME_INTERNAL_STATUS_AUV $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
cp $SERVICE_FILE_PATH_BLACKBOX$SERVICE_FILE_NAME_BLACKBOX $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Replace placeholders in the .service file
# Note: This assumes <pathToThisFile> is to be replaced with the current directory
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_LCD|g" $SERVICE_FILE_NAME_LCD # LCD
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_INTERNAL_STATUS_AUV|g" $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
sed -i "s|<pathToThisFile>|$SCRIPT_DIR$SERVICE_FILE_PATH_BLACKBOX|g" $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Place in the correct python dependency for the scripts that service file executes
sed -i "s|<PythonVersion>|$PYTHON_VERSION|g" $SERVICE_FILE_NAME_LCD
sed -i "s|<PythonVersion>|$PYTHON_VERSION|g" $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV

# Kill the systems service file if it exists
sudo systemctl kill $SERVICE_FILE_NAME_LCD # LCD
sudo systemctl kill $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
sudo systemctl kill $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Copy the modified .service file to the systemd directory
# Note: Need sudo permission to copy to /etc/systemd/system
sudo cp $SERVICE_FILE_NAME_LCD $SYSTEMD_PATH # LCD
sudo cp $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV $SYSTEMD_PATH # Internal Status
sudo cp $SERVICE_FILE_NAME_BLACKBOX $SYSTEMD_PATH # Blackbox

# Delete the redundant copy
rm $SERVICE_FILE_NAME_LCD # LCD
rm $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
rm $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Change permission of the .service file
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_LCD # LCD
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
sudo chmod 777 $SYSTEMD_PATH$SERVICE_FILE_NAME_BLACKBOX # Blackbox



# Start .service files -----------
echo "Starting .service files..."
# Reload systemd to recognize the new services
sudo systemctl daemon-reload

# Enable services to start on boot
sudo systemctl enable $SERVICE_FILE_NAME_LCD # LCD
sudo systemctl enable $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
sudo systemctl enable $SERVICE_FILE_NAME_BLACKBOX # Blackbox

# Start the service immediately
sudo systemctl start $SERVICE_FILE_NAME_LCD # LCD
sudo systemctl start $SERVICE_FILE_NAME_INTERNAL_STATUS_AUV # Internal Status
sudo systemctl start $SERVICE_FILE_NAME_BLACKBOX # Blackbox



# Debugging ----------
echo "'$SERVICE_FILE_NAME_LCD' - installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_INTERNAL_STATUS_AUV' - installed and started successfully :)"
echo "'$SERVICE_FILE_NAME_BLACKBOX' - installed and started successfully :)"
