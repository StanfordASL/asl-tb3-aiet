#!/bin/bash

# =============================================================================
# Laptop Setup Script (Robot-paired)
# Run with: sudo ./setup_laptop.sh
# =============================================================================

set -e

# -----------------------------------------------------------------------------
# ROBOT CONFIGURATION (must match robot setup)
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

# -----------------------------------------------------------------------------
# PATHS
# -----------------------------------------------------------------------------
REAL_USER="$(logname)"
HOME_DIR="/home/$REAL_USER"
WS_DIR="$HOME_DIR/autonomy_ws"
REPO_DIR="$WS_DIR/src/asl-tb3-aiet"
BASHRC="$HOME_DIR/.bashrc"

# -----------------------------------------------------------------------------
# HELPERS
# -----------------------------------------------------------------------------


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

check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "Run as root: sudo ./setup_laptop.sh"
        exit 1
    fi
}

list_robots() {
    echo "Available robots:"
    for r in "${ROBOTS[@]}"; do
        echo "  - $(echo "$r" | cut -d':' -f1)"
    done
}

get_robot_config() {
    local name="$1"
    for r in "${ROBOTS[@]}"; do
        if [ "$(echo "$r" | cut -d':' -f1)" = "$name" ]; then
            echo "$r"
            return 0
        fi
    done
    return 1
}

# -----------------------------------------------------------------------------
# SETUP STEPS
# -----------------------------------------------------------------------------
forget_all_wifi() {
    print_info "Forgetting all existing WIFI connections..."
    nmcli -t -f NAME connection show | while IFS= read -r c; do 
         if nmcli -g connection.type connection show "$c" | grep -q 802-11-wireless; then
              nmcli connection delete "$c"
         fi
    done
    print_success "Successfully forgot all existing WIFI connections."
}

connect_wifi() {
    print_info "Attempting to connect to wifi..."
    local ssid="$1"
    local password="$2"

    nmcli radio wifi on
    sleep 2
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

update_system() {
    print_info "Installing Apt packages..."
    apt update
}

install_terminator() {
    apt install -y terminator
    print_success "Apt packages installed."
}

update_repo() {
    print_info "Updating asl-tb3-* repositories..."
    sudo -u "$REAL_USER" bash <<EOF
rm -rf "$WS_DIR/src"
mkdir "$WS_DIR/src"
cd "$WS_DIR/src"
git clone https://github.com/StanfordASL/asl-tb3-aiet.git
cd asl-tb3-aiet
git checkout tps-2026
cd "$WS_DIR/src"
git clone https://github.com/StanfordASL/asl-tb3-driver.git
cd asl-tb3-driver
git checkout tps-2026
cd "$WS_DIR/src"
git clone https://github.com/StanfordASL/asl-tb3-utils.git
EOF
    print_success "All repositories updated."
}

clean_local_folders() {
    print_info "Removing old files in section_assests and Downloads."
    rm -rf "$HOME_DIR/section_assets/*"
    rm -rf "$HOME_DIR/Downloads/*"
    print_success "old files in section_assests and Downloads removed."
}

install_python_packages() {
    print_info "Installing python vitual environment."
    sudo -u "$REAL_USER" bash <<EOF
cd "$REPO_DIR"
python3 -m venv ros_env
source ros_env/bin/activate
pip install -r requirements-portable.txt
EOF
    print_success "Pip packages installed."
}

update_bashrc() {
    print_info "Updating .bashrc..."
    local robot="$1"
    local domain="$2"

    cp /media/aa274/OrwellBackB/.bashrc "$BASHRC"

    sed -i '/^export LAPTOP_ID=/d' "$BASHRC"
    sed -i '/^export ROS_DOMAIN_ID=/d' "$BASHRC"
    sed -i '/autonomy_ws\/src\/asl-tb3-aiet\/ros_env\/bin\/activate/d' "$BASHRC"
    sed -i '/autonomy_ws\/install\/setup.bash/d' "$BASHRC"

    {
        echo "export LAPTOP_ID='${robot}-laptop'"
        echo "export ROS_DOMAIN_ID=$domain"
        echo "source ~/autonomy_ws/src/asl-tb3-aiet/ros_env/bin/activate"
        echo "source ~/autonomy_ws/install/setup.bash"
    } >> "$BASHRC"
    print_success ".bashrc updated."
}

build_workspace() {
    print_info "Building workspace."
    sudo -u "$REAL_USER" bash <<EOF
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
rm -rf build install log
colcon build --symlink-install
EOF
    print_success "Workspace built."
}

# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------
main() {
    check_root

    list_robots
    echo ""
    read -p "Enter robot name to pair with: " ROBOT_NAME

    CONFIG=$(get_robot_config "$ROBOT_NAME") || {
        echo "Robot not found"
        exit 1
    }

    ROS_DOMAIN_ID=$(echo "$CONFIG" | cut -d':' -f2)
    WIFI_SSID=$(echo "$CONFIG" | cut -d':' -f3)
    WIFI_PASS=$(echo "$CONFIG" | cut -d':' -f4)

    echo ""
    echo "Resolved configuration:"
    echo "  Robot:         $ROBOT_NAME"
    echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "  WiFi SSID:     $WIFI_SSID"
    echo ""

    read -p "Proceed with setup? (y/n): " CONFIRM
    [ "$CONFIRM" = "y" ] || exit 0

    forget_all_wifi
    connect_wifi "$WIFI_SSID" "$WIFI_PASS"
    update_system
    install_terminator
    update_repo
    clean_local_folders
    install_python_packages
    update_bashrc "$ROBOT_NAME" "$ROS_DOMAIN_ID"
    build_workspace

    echo "Setup complete. Run: source ~/.bashrc"
}

main "$@"
