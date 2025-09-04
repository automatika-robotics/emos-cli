#!/bin/bash

# ==============================================================================
# emos - EmbodiedOS Management CLI v0.1.0
# ==============================================================================

# --- Configuration ---
EMOS_VERSION="0.1.0"
CONFIG_DIR="$HOME/.config/emos"
LICENSE_FILE="$CONFIG_DIR/license.key"
CONTAINER_NAME="emos"
SERVICE_NAME="emos.service"
API_ENDPOINT="https://support.automatika-robotics.com/v1/license/validate"
INSTALLER_URL="https://raw.githubusercontent.com/automatika-robotics/emos-cli/main/install.sh"

# --- Styling and UI Functions ---
display_art() {
    ART=$(cat << "EOF"
███████╗███╗   ███╗ ██████╗ ███████╗
██╔════╝████╗ ████║██╔═══██╗██╔════╝
█████╗  ██╔████╔██║██║   ██║███████╗
██╔══╝  ██║╚██╔╝██║██║   ██║╚════██║
███████╗██║ ╚═╝ ██║╚██████╔╝███████║
╚══════╝╚═╝     ╚═╝ ╚═════╝ ╚══════╝
EOF
)
    gum style --border normal --border-foreground 212 --padding "1 2" "$ART"
    gum style --foreground 212 "EmbodiedOS Management CLI v${EMOS_VERSION}"
    echo
}
run_with_spinner() {
    local title="$1"
    local cmd="$2"
    OUTPUT=$(eval "$cmd" 2>&1)
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 0 ]; then
        gum spin --spinner dot --title "$title" --show-output -- sleep 1.5
        gum style --foreground 2 "✔ Success:" "$title"
        return 0
    else
        gum spin --spinner dot --title "$title" --show-output -- sleep 0.5
        gum style --foreground 1 "✖ Error:" "$title"
        gum style --foreground 1 "  Please check the output below for details."
        echo "$OUTPUT"
        return 1
    fi
}

# --- Core Logic Functions ---
check_dependencies() {
    local missing_deps=()
    for dep in gum docker curl jq; do
        if ! command -v "$dep" &> /dev/null; then
            missing_deps+=("$dep")
        fi
    done
    if [ ${#missing_deps[@]} -ne 0 ]; then
        gum style --foreground 1 "Error: Missing required dependencies: ${missing_deps[*]}. Please install them to continue."
        exit 1
    fi
}

show_help() {
    gum style --bold "EMOS Management CLI"
    echo "Usage: emos [command]"
    echo
    gum style --bold "Available Commands:"
    gum table << EOF
COMMAND,DESCRIPTION
"install <key>", "Install and start EMOS using a license key."
"update", "Update the CLI and/or the EMOS container."
"status", "Display info and container status."
"--help", "Show this help message."
"--version", "Show the current version of the CLI tool."
EOF
}

show_status() {
    display_art
    echo "EMOS is a self-contained automation layer for your robot. Developed with ❤️ by Automatika."
    echo "This tool helps you manage its lifecycle on this machine."
    echo
    gum style --bold "Container Status:"
    STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)
    if [ "$STATUS" == "running" ]; then
        STATUS_MSG=$(gum style --foreground 2 "● Running")
    elif [ "$STATUS" == "exited" ]; then
        STATUS_MSG=$(gum style --foreground 3 "● Exited")
    else
        STATUS_MSG=$(gum style --foreground 1 "● Not Found / Not Running")
    fi
    echo -e "  - Container '$CONTAINER_NAME': $STATUS_MSG"
    if [ -f "$LICENSE_FILE" ]; then
        echo -e "  - License Key: $(gum style --foreground 2 "Found")"
    else
        echo -e "  - License Key: $(gum style --foreground 1 "Not Found")"
    fi
}

do_install() {
    local license_key="$1"
    if [ -z "$license_key" ]; then
        gum style --foreground 1 "✖ Error: A license key is required."
        echo "Usage: emos install <your_license_key>"
        exit 1
    fi
    display_art
    echo "🚀 Starting EMOS installation..."
    mkdir -p "$CONFIG_DIR"
    gum spin --spinner dot --title "Validating license key..." -- \
    API_RESPONSE=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{\"key\": \"$license_key\"}" \
        "$API_ENDPOINT")
    if [ -z "$API_RESPONSE" ] || ! echo "$API_RESPONSE" | jq empty; then
        gum style --foreground 1 "✖ Error: Failed to contact API or received an invalid response."
        exit 1
    fi
    if echo "$API_RESPONSE" | jq -e '.error' > /dev/null; then
        ERROR_MSG=$(echo "$API_RESPONSE" | jq -r '.error')
        gum style --foreground 1 "✖ API Error: $ERROR_MSG"
        exit 1
    fi
    gum style --foreground 2 "✔ License key validated successfully."
    DOCKER_REGISTRY=$(echo "$API_RESPONSE" | jq -r '.registry')
    DOCKER_IMAGE=$(echo "$API_RESPONSE" | jq -r '.image')
    DOCKER_USER=$(echo "$API_RESPONSE" | jq -r '.username')
    DOCKER_TOKEN=$(echo "$API_RESPONSE" | jq -r '.token')

    # Store only the license key, not the credentials.
    echo "$license_key" > "$LICENSE_FILE"
    # REMOVED: The line saving API_RESPONSE to DOCKER_CREDS_FILE is gone.

    FULL_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_IMAGE}"
    run_with_spinner "Logging into Docker registry..." \
        "echo \"$DOCKER_TOKEN\" | docker login \"$DOCKER_REGISTRY\" -u \"$DOCKER_USER\" --password-stdin" || exit 1
    run_with_spinner "Pulling EmbodiedOS container image..." \
        "docker pull \"$FULL_IMAGE_NAME\"" || exit 1
    run_with_spinner "Starting EmbodiedOS container..." \
        "docker run -d --restart always --name \"$CONTAINER_NAME\" -p 8080:80 \"$FULL_IMAGE_NAME\"" || exit 1
    # ... (systemd service creation is unchanged) ...
    gum style --bold --foreground 212 "\nCreating systemd service for auto-restart..."
    echo "This step requires administrative privileges to create a system service file."
    SERVICE_FILE_CONTENT=$(cat <<EOF
[Unit]
Description=EmbodiedOS Container
After=docker.service
Requires=docker.service
[Service]
Restart=always
ExecStart=/usr/bin/docker start -a $CONTAINER_NAME
ExecStop=/usr/bin/docker stop -t 2 $CONTAINER_NAME
[Install]
WantedBy=multi-user.target
EOF
)
    if gum confirm "Create /etc/systemd/system/${SERVICE_NAME}?"; then
        echo "$SERVICE_FILE_CONTENT" | sudo tee "/etc/systemd/system/${SERVICE_NAME}" > /dev/null
        run_with_spinner "Reloading systemd daemon..." "sudo systemctl daemon-reload" || exit 1
        run_with_spinner "Enabling emos service..." "sudo systemctl enable ${SERVICE_NAME}" || exit 1
    else
        gum style --foreground 3 "Skipping systemd service creation."
    fi
    gum style --border double --padding "1 5" --border-foreground 2 "🎉 EmbodiedOS installed successfully! 🎉"
}

do_update() {
    display_art
    gum spin --spinner dot --title "Checking for emos CLI updates..." -- sleep 1
    echo "The update process requires administrative privileges to check/update the CLI."
    set -o pipefail
    curl -sSL "$INSTALLER_URL" | sudo bash -s -- update
    CLI_UPDATE_STATUS=$?
    set +o pipefail
    if [ "$CLI_UPDATE_STATUS" -eq 10 ]; then
        gum style --foreground 212 "💡 The emos CLI has been updated to the latest version."
        gum style --bold "Please run 'emos update' again to update your EMOS container."
        exit 0
    elif [ "$CLI_UPDATE_STATUS" -ne 0 ]; then
        gum style --foreground 1 "✖ Error: The CLI update check failed. Cannot proceed."
        exit 1
    fi
    gum style --foreground 2 "✔ Your emos CLI is up to date."
    echo

    # Check for the license file.
    if [ ! -f "$LICENSE_FILE" ]; then
        gum style --foreground 1 "✖ Error: No existing installation found."
        echo "Please run 'emos install <license_key>' first."
        exit 1
    fi

    echo "🚀 Checking for EMOS container updates..."
    LICENSE_KEY=$(cat "$LICENSE_FILE")

    # Re-validate the license to get fresh credentials.
    gum spin --spinner dot --title "Verifying license for update..." -- \
    API_RESPONSE=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{\"key\": \"$LICENSE_KEY\"}" \
        "$API_ENDPOINT")
    if echo "$API_RESPONSE" | jq -e '.error' > /dev/null; then
        ERROR_MSG=$(echo "$API_RESPONSE" | jq -r '.error')
        gum style --foreground 1 "✖ API Error: $ERROR_MSG"
        exit 1
    fi
    gum style --foreground 2 "✔ License verified."
    DOCKER_REGISTRY=$(echo "$API_RESPONSE" | jq -r '.registry')
    DOCKER_IMAGE=$(echo "$API_RESPONSE" | jq -r '.image')
    DOCKER_USER=$(echo "$API_RESPONSE" | jq -r '.username')
    DOCKER_TOKEN=$(echo "$API_RESPONSE" | jq -r '.token')
    FULL_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_IMAGE}"

    run_with_spinner "Stopping the current container..." \
        "docker stop \"$CONTAINER_NAME\""
    run_with_spinner "Removing the old container..." \
        "docker rm \"$CONTAINER_NAME\""
    run_with_spinner "Logging into Docker registry..." \
        "echo \"$DOCKER_TOKEN\" | docker login \"$DOCKER_REGISTRY\" -u \"$DOCKER_USER\" --password-stdin" || exit 1
    run_with_spinner "Pulling the latest EmbodiedOS image..." \
        "docker pull \"$FULL_IMAGE_NAME\"" || exit 1
    run_with_spinner "Starting the new EmbodiedOS container..." \
        "docker run -d --restart always --name \"$CONTAINER_NAME\" -p 8080:80 \"$FULL_IMAGE_NAME\"" || exit 1
    gum style --border double --padding "1 5" --border-foreground 2 "✅ EmbodiedOS container updated successfully! ✅"
}

# --- Main Execution ---
main() {
    check_dependencies
    case "$1" in
        install)
            do_install "$2"
            ;;
        update)
            do_update
            ;;
        status)
            show_status
            ;;
        --version)
            echo "emos version $EMOS_VERSION"
            ;;
        --help)
            show_help
            ;;
        "")
            show_help
            ;;
        *)
            gum style --foreground 1 "✖ Error: Unknown command '$1'"
            echo "Run 'emos --help' to see a list of available commands."
            exit 1
            ;;
    esac
}
main "$@"
