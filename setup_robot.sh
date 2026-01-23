#!/bin/bash

# =============================================================================
# Robot Setup Script
# Run with: sudo ./setup_robot.sh
# =============================================================================

# -----------------------------------------------------------------------------
# ROBOT CONFIGURATION - Add your robots here
# Format: "robot_name:ros_domain_id:wifi_ssid:wifi_password"
# -----------------------------------------------------------------------------
ROBOTS=(
    "goldie:1:NETGEAR28-5G-1:purplehippo894"
    "rosie:5:NETGEAR28-5G-1:purplehippo894"
    "montague:10:NETGEAR28-5G-1:purplehippo894"
    "stella:15:Linksys1_5GHz:2024!CARS"
    "astro:20:Linksys1_5GHz:2024!CARS"
    "cogswell:25:NETGEAR59-5G-1:redpiano286"
    "henry:30:NETGEAR59-5G-1:redpiano286"
    "lulu:35:Linksys3_5GHz:2024!CARS"
    "teddy:40:Linksys3_5GHz:2024!CARS"
    "marcia:45:NETGEAR58-5G-1:greenwindow844"
    "curly:50:Linksys4_5GHz:2024!CARS"
    "scraps:55:NETGEAR12-5G:oddcurtain147"
    "asteroid:60:NETGEAR45-5G:melodicdiamond006"
    "sara:65:NETGEAR58-5G-1:greenwindow844"
    "judy:70:Linksys4_5GHz:2024!CARS"
    "galaxy:75:NETGEAR12-5G:oddcurtain147"
    "sparky:80:NETGEAR45-5G:melodicdiamond006"
    "lucy:85:NETGEAR28-5G-1:purplehippo894"
    "arthur:90:Linksys1_5GHz:2024!CARS"
    "harlan:100:NETGEAR59-5G-1:redpiano286"
    "orwell:105:Linksys3_5GHz:2024!CARS"
)

# Velodyne configuration
VELODYNE_IFACE="eth0"
VELODYNE_NEW_NAME="velodyne0"
VELODYNE_TARGET_IP="192.168.1.100"
VELODYNE_NETMASK="24"
VELODYNE_SENSOR_IP="192.168.1.201"


# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Check for root privileges
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "Please run as root (sudo ./setup_robot.sh)"
        exit 1
    fi
}

# Get the actual user who ran sudo
get_real_user() {
    if [ -n "$SUDO_USER" ]; then
        echo "$SUDO_USER"
    else
        echo "$USER"
    fi
}

# Look up robot configuration
get_robot_config() {
    local robot_name="$1"
    for robot in "${ROBOTS[@]}"; do
        local name=$(echo "$robot" | cut -d':' -f1)
        if [ "$name" == "$robot_name" ]; then
            echo "$robot"
            return 0
        fi
    done
    return 1
}

# List available robots
list_robots() {
    echo "Available robots:"
    for robot in "${ROBOTS[@]}"; do
        local name=$(echo "$robot" | cut -d':' -f1)
        local domain_id=$(echo "$robot" | cut -d':' -f2)
        echo "  - $name (ROS_DOMAIN_ID: $domain_id)"
    done
}

# -----------------------------------------------------------------------------
# Setup Functions
# -----------------------------------------------------------------------------

reset_machine_id() {
    print_header "Resetting Machine ID"

    rm -f /etc/machine-id
    systemd-machine-id-setup

    print_success "Machine ID reset complete"
    print_info "New machine ID: $(cat /etc/machine-id)"
}

set_hostname() {
    local hostname="$1"
    print_header "Setting Hostname to: $hostname"

    hostnamectl set-hostname "$hostname"

    # Verify
    print_info "Verification:"
    echo "  hostname: $(hostname)"
    echo "  /etc/hostname: $(cat /etc/hostname)"

    print_success "Hostname set successfully"
}

regenerate_ssh_keys() {
    print_header "Regenerating SSH Host Keys"

    rm -f /etc/ssh/ssh_host_*
    dpkg-reconfigure openssh-server

    print_success "SSH host keys regenerated"
}

forget_all_wifi() {
    print_info "Forgetting all existing WIFI connections..."
    nmcli -t -f NAME connection show | while IFS= read -r c; do 
         if nmcli -g connection.type connection show "$c" | grep -q 802-11-wireless; then
              nmcli connection delete "$c"
         fi
    done
    print_success "Successfully forgot all existing WIFI connections."
}

setup_wifi() {
    local ssid="$1"
    local password="$2"

    print_header "Setting Up WiFi Connection"

    # Check WiFi device status
    print_info "Checking WiFi device status..."
    nmcli device status

    # Enable WiFi
    print_info "Enabling WiFi..."
    nmcli radio wifi on
    sleep 2

    # Scan for networks
    print_info "Scanning for networks..."
    nmcli device wifi list

    # Connect to network
    print_info "Connecting to $ssid..."
    nmcli device wifi connect "$ssid" password "$password"

    if [ $? -eq 0 ]; then
        print_success "Connected to $ssid"

        # Configure connection settings
        print_info "Configuring connection settings..."
        nmcli connection modify "$ssid" connection.permissions ""
        nmcli connection modify "$ssid" connection.autoconnect yes
        nmcli connection modify "$ssid" connection.autoconnect-priority 100

        # Verify connection
        print_info "Verifying connection..."
        nmcli connection show --active
        ip addr show | grep -A 5 "wl"

        print_success "WiFi setup complete"
    else
        print_error "Failed to connect to $ssid"
        print_warning "You may need to connect manually"
    fi
}

setup_avahi() {
    local hostname="$1"
    print_header "Setting Up Avahi (.local resolution)"

    # Install avahi if needed
    print_info "Ensuring Avahi is installed..."
    apt update -qq
    apt install -y avahi-daemon avahi-utils

    # Enable and start Avahi
    print_info "Enabling and starting Avahi daemon..."
    systemctl enable avahi-daemon
    systemctl start avahi-daemon
    systemctl restart avahi-daemon

    # Check status
    print_info "Checking Avahi status..."
    systemctl status avahi-daemon --no-pager

    print_success "Avahi setup complete"
    print_info "Robot should be accessible at: ${hostname}.local"
}

setup_ros_domain_id() {
    local domain_id="$1"
    local real_user=$(get_real_user)
    local bashrc_path="/home/$real_user/.bashrc"

    print_header "Setting ROS_DOMAIN_ID to: $domain_id"

    # Remove any existing ROS_DOMAIN_ID lines
    sed -i '/^export ROS_DOMAIN_ID/d' "$bashrc_path"

    # Add new ROS_DOMAIN_ID
    echo "export ROS_DOMAIN_ID=$domain_id" >> "$bashrc_path"

    print_success "ROS_DOMAIN_ID added to $bashrc_path"
    print_info "Run 'source ~/.bashrc' or reboot to apply"
}

setup_velodyne() {
    print_header "Setting Up Velodyne Interface"

    # Check if interface exists
    if ip link show "$VELODYNE_IFACE" > /dev/null 2>&1; then
        MAC_ADDR=$(cat /sys/class/net/$VELODYNE_IFACE/address)
        print_info "Found $VELODYNE_IFACE with MAC: $MAC_ADDR"
    else
        print_warning "Interface $VELODYNE_IFACE not found"
        print_info "Available interfaces:"
        ip link show
        print_warning "Skipping Velodyne setup - you may need to configure manually"
        return 1
    fi

    # Create udev rule for persistence
    RULE_FILE="/etc/udev/rules.d/70-persistent-net.rules"
    print_info "Writing udev rule to $RULE_FILE..."
    echo "SUBSYSTEM==\"net\", ACTION==\"add\", DRIVERS==\"?*\", ATTR{address}==\"$MAC_ADDR\", ATTR{type}==\"1\", NAME=\"$VELODYNE_NEW_NAME\"" > $RULE_FILE

    # Apply changes immediately
    print_info "Applying network changes..."
    ip link set "$VELODYNE_IFACE" down
    ip link set "$VELODYNE_IFACE" name "$VELODYNE_NEW_NAME"
    ip link set "$VELODYNE_NEW_NAME" up

    # Stop NetworkManager from interfering
    if command -v nmcli >/dev/null 2>&1; then
        print_info "Configuring NetworkManager to ignore $VELODYNE_NEW_NAME..."
        nmcli device set "$VELODYNE_NEW_NAME" managed no
    fi

    # Set IP address
    ip addr flush dev "$VELODYNE_NEW_NAME"
    ip addr add "$VELODYNE_TARGET_IP/$VELODYNE_NETMASK" dev "$VELODYNE_NEW_NAME"

    print_info "Interface renamed to $VELODYNE_NEW_NAME and IP set to $VELODYNE_TARGET_IP"

    # Verification
    print_info "Verifying setup..."
    ip addr show "$VELODYNE_NEW_NAME"

    print_info "Attempting to ping Velodyne sensor at $VELODYNE_SENSOR_IP..."
    if ping -c 3 -W 1 $VELODYNE_SENSOR_IP; then
        print_success "Sensor is reachable!"
    else
        print_warning "Sensor not reachable. Check physical connection or sensor power."
    fi
}


# -----------------------------------------------------------------------------
# Main Script
# -----------------------------------------------------------------------------

main() {
    check_root

    echo ""
    echo "=============================================="
    echo "       Robot Setup Script"
    echo "=============================================="
    echo ""

    # List available robots
    list_robots
    echo ""

    # Prompt for robot name
    read -p "Enter robot name: " ROBOT_NAME

    # Look up robot configuration
    CONFIG=$(get_robot_config "$ROBOT_NAME")

    if [ -z "$CONFIG" ]; then
        print_error "Robot '$ROBOT_NAME' not found in configuration"
        print_info "Please add this robot to the ROBOTS array at the top of the script"
        exit 1
    fi

    # Parse configuration
    ROS_DOMAIN_ID=$(echo "$CONFIG" | cut -d':' -f2)
    WIFI_SSID=$(echo "$CONFIG" | cut -d':' -f3)
    WIFI_PASSWORD=$(echo "$CONFIG" | cut -d':' -f4)

    # Show configuration
    echo ""
    print_info "Configuration for $ROBOT_NAME:"
    echo "  ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
    echo "  WiFi SSID:      $WIFI_SSID"
    echo "  WiFi Password:  ${WIFI_PASSWORD:0:3}***"
    echo ""

    read -p "Proceed with setup? (y/n): " CONFIRM
    if [ "$CONFIRM" != "y" ] && [ "$CONFIRM" != "Y" ]; then
        echo "Setup cancelled"
        exit 0
    fi

    # Run setup steps
    reset_machine_id
    set_hostname "$ROBOT_NAME"
    regenerate_ssh_keys
    forget_all_wifi
    setup_wifi "$WIFI_SSID" "$WIFI_PASSWORD"
    setup_avahi "$ROBOT_NAME"
    setup_ros_domain_id "$ROS_DOMAIN_ID"
    setup_velodyne

    # Final summary
    print_header "Setup Complete!"
    echo ""
    print_info "Summary:"
    echo "  Hostname:       $ROBOT_NAME"
    echo "  ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
    echo "  WiFi:           $WIFI_SSID"
    echo "  mDNS address:   ${ROBOT_NAME}.local"
    echo ""
    print_warning "A REBOOT is required to complete the setup"
    echo ""

    read -p "Reboot now? (y/n): " REBOOT_CONFIRM
    if [ "$REBOOT_CONFIRM" == "y" ] || [ "$REBOOT_CONFIRM" == "Y" ]; then
        print_info "Rebooting in 5 seconds..."
        sleep 5
        reboot
    else
        print_info "Please reboot manually when ready: sudo reboot"
    fi
}

# Run main function
main "$@"
