#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status.

# --- Configuration ---
EMOS_CLI_URL="https://raw.githubusercontent.com/automatika-robotics/emos-cli/main/emos-cli.sh"
RECIPE_RUN_URL="https://raw.githubusercontent.com/automatika-robotics/emos-cli/main/recipe_run.sh"
INSTALL_PATH="/usr/local/bin/emos"

# --- UI Functions ---
info() {
    echo "[INFO] $1"
}

success() {
    printf "\033[0;32m[SUCCESS] %s\033[0m\n" "$1"
}

error() {
    printf "\033[0;31m[ERROR] %s\033[0m\n" "$1" >&2
    exit 1
}

# --- Core Logic Functions ---
check_root() {
    if [ "$EUID" -ne 0 ]; then
        error "This script must be run with sudo or as the root user."
    fi
}

# Check if Docker is installed, as it's a prerequisite.
check_docker() {
    if ! command -v docker &> /dev/null; then
        error "Docker is not installed. Please install Docker before running this script. See: https://docs.docker.com/engine/install/"
    fi
    info "Docker installation found."
}

detect_package_manager() {
    if command -v apt-get &>/dev/null; then
        echo "apt"
    elif command -v dnf &>/dev/null; then
        echo "dnf"
    elif command -v yum &>/dev/null; then
        echo "yum"
    elif command -v pacman &>/dev/null; then
        echo "pacman"
    elif command -v brew &>/dev/null; then
        echo "brew"
    else
        echo "unknown"
    fi
}

# Install dependencies
install_dependencies() {
    info "Detecting package manager..."
    PKG_MANAGER=$(detect_package_manager)

    info "Installing dependencies..."
    case "$PKG_MANAGER" in
        apt)
            # Install base dependencies first (including gpg for key management)
            info "Installing: curl, jq, gpg..."
            apt-get update
            apt-get install -y curl jq gpg

            # Set up Charm repository specifically for gum
            info "Setting up Charm repository to install gum..."
            mkdir -p /etc/apt/keyrings
            curl -fsSL https://repo.charm.sh/apt/gpg.key | gpg --dearmor > /etc/apt/keyrings/charm.gpg
            echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" | tee /etc/apt/sources.list.d/charm.list

            apt-get update
            apt-get install -y gum
            ;;
        dnf|yum)
            info "Installing: gum, curl, jq..."
            dnf install -y gum curl jq
            ;;
        pacman)
            info "Installing: gum, curl, jq..."
            pacman -Syu --noconfirm gum curl jq
            ;;
        brew)
            info "Installing: gum, curl, jq..."
            brew install gum curl jq
            ;;
        *)
            error "Unsupported package manager. Please install dependencies manually: gum, curl, and jq."
            ;;
    esac
    success "Dependencies installed."
}

# Download and install the latest version of the 'emos' CLI
install_cli() {
    info "Downloading the latest emos CLI tools..."
    
    # Use chained curl commands with -f to ensure both must succeed.
    if curl -sSLf "$EMOS_CLI_URL" -o "/tmp/emos" && \
       curl -sSLf "$RECIPE_RUN_URL" -o "/tmp/recipe_run.sh"; then
        
        info "Installing emos to $INSTALL_PATH/emos..."
        mv "/tmp/emos" "$INSTALL_PATH/emos"
        chmod +x "$INSTALL_PATH/emos"
        
        info "Installing recipe_run.sh to $INSTALL_PATH/recipe_run.sh..."
        mv "/tmp/recipe_run.sh" "$INSTALL_PATH/recipe_run.sh"
        chmod +x "$INSTALL_PATH/recipe_run.sh"
        
        success "emos CLI tools installed successfully."
    else
        error "Failed to download one or more CLI tools. Check the URLs and your connection."
    fi
}

# Checks for an update and performs it if necessary
update_cli() {
    info "Checking for emos CLI updates..."

    if [ ! -f "$INSTALL_PATH/emos" ]; then
        error "emos is not installed. Cannot update."
    fi

    local remote_version
    remote_version=$(curl -sSLf "$EMOS_CLI_URL" | grep '^EMOS_VERSION=' | cut -d'=' -f2 | tr -d '"')
    if [ -z "$remote_version" ]; then
        error "Could not determine the latest version available for download."
    fi

    local local_version
    local_version=$(grep '^EMOS_VERSION=' "$INSTALL_PATH/emos" | cut -d'=' -f2 | tr -d '"')

    info "Local version: $local_version, Remote version: $remote_version"

    if [ "$remote_version" != "$local_version" ]; then
        info "A new version ($remote_version) is available. Updating all tools..."
        install_cli 
        # This special exit code will be caught by the 'emos update' command
        exit 10
    else
        info "You already have the latest version of the emos CLI."
        exit 0
    fi
}

# --- Main Execution ---
main() {
    check_root

    case "$1" in
        # Called by 'emos update'
        update)
            update_cli
            ;;
        # Called by user for first-time install
        "")
            info "Starting first-time installation of emos..."
            check_docker
            install_dependencies
            install_cli
            info "Run 'emos --help' to get started."
            ;;
        *)
            error "Unknown command for installer: '$1'"
            ;;
    esac
}

main "$@"
