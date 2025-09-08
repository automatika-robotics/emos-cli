#!/bin/bash

# ==============================================================================
# emos - EmbodiedOS Management CLI v0.1.1
# ==============================================================================

# --- Configuration ---
EMOS_VERSION="0.1.1"
CONFIG_DIR="$HOME/.config/emos"
LICENSE_FILE="$CONFIG_DIR/license.key"
CONTAINER_NAME="emos"
SERVICE_NAME="emos.service"
DOCKER_RUN_CMD="docker run -d -it --restart always --privileged -v /dev/bus/usb:/dev/bus/usb -v $HOME/emos:/emos --name \"\$CONTAINER_NAME\" --network host --runtime nvidia --gpus=all \"\$FULL_IMAGE_NAME\""
API_ENDPOINT="https://support-api.automatikarobotics.com/api/registrations/ghcr-credentials"
INSTALLER_URL="https://raw.githubusercontent.com/automatika-robotics/emos-cli/main/install.sh"
# --- Theme ---
THEME_RED="#d54e53"
THEME_BLUE="#81a2be"
THEME_NEUTRAL="#EEF2F3"

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
    gum style --foreground $THEME_RED --padding "1 2" "$ART"
    gum style --padding "0 2" --foreground $THEME_BLUE --bold "EmbodiedOS Management CLI v${EMOS_VERSION}"
    echo
}

run_with_spinner() {
    local title="$1"
    local cmd="$2"
    local OUTPUT
    local EXIT_CODE

    # use `eval` to correctly handle commands with pipes and quotes.
    # `2>&1` redirects stderr to stdout
    OUTPUT=$(eval "$cmd" 2>&1)
    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        # On success, show a brief spinner animation.
        gum spin --spinner dot --title "$title" -- sleep 1
        gum style --foreground 2 "✔ Success:" "$title"
        [ -n "$OUTPUT" ] && echo "$OUTPUT"
        return 0
    else
        # On failure, print our error header AND the captured output.
        gum style --foreground 1 "✖ Error:" "$title"
        gum style --faint "  The command failed with the following output:"
        # Use gum format to indent the error message nicely.
        gum format -- "$OUTPUT"
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
    gum style --bold --foreground $THEME_BLUE "EmbodiedOS Management CLI"
    echo "Usage: emos [command]"
    echo

    # Format the options to look like a table for the interactive menu
    gum style --bold --foreground $THEME_BLUE "? Select a command to generate a template:"
    local choice
    choice=$(gum choose --height 7 --cursor-prefix "➜ " --header " " \
        --item.foreground $THEME_NEUTRAL \
        --cursor.foreground $THEME_RED \
        "install   - Install and start EMOS using a license key." \
        "update    - Update the CLI and/or the EMOS container." \
        "status    - Display info and container status." \
        "version   - Show the current version of the CLI tool." \
        "exit      - Exit this menu.")

    # Check if the user made a choice (didn't press ESC)
    if [ -n "$choice" ]; then
        # Extract the command keyword from the choice (the first word)
        local command_keyword
        command_keyword=$(echo "$choice" | awk '{print $1}')

        case "$command_keyword" in
            "install")
                gum style --faint "Voilà ! Copy, add license key and press Enter."
                printf "emos install <license-key>\n"
                ;;
            "update")
                gum style --faint "Voilà ! Copy and press Enter."
                printf "emos update\n"
                ;;
            "status")
                gum style --faint "Voilà ! Copy and press Enter."
                printf "emos status\n"
                ;;
            "version")
                gum style --faint "Voilà ! Copy and press Enter."
                printf "emos --version\n"
                ;;
            "exit")
                # Just exit cleanly with no output
                exit 0
                ;;
        esac
    fi
}
show_status() {
    display_art
    echo "EMOS is a self-contained automation layer for your robot."
    echo "This tool helps you manage its lifecycle on this machine."
    echo "Developed with ❤️  by Automatika Robotics."
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

# Validates a license key via the API and returns the JSON response.
# Exits with an error if validation fails.
# Usage: local API_RESPONSE=$(api_validate_license "<key>")
api_validate_license() {
    local license_key="$1"
    local TMP_FILE
    TMP_FILE=$(mktemp)

    if ! gum spin --spinner dot --title "Validating license key..." -- \
        curl -s -X POST \
            -H "Content-Type: application/json" \
            -d "{\"license_key\": \"$license_key\"}" \
            "$API_ENDPOINT" -o "$TMP_FILE"; then
        rm "$TMP_FILE"
        gum style --foreground 1 "✖ Error: Could not connect to the license API. Check your network connection."
        exit 1
    fi

    local api_response
    api_response=$(cat "$TMP_FILE")
    rm "$TMP_FILE"

    if [ -z "$api_response" ] || ! echo "$api_response" | jq empty; then
        gum style --foreground 1 "✖ Error: API returned an invalid or empty response."
        exit 1
    fi
    if echo "$api_response" | jq -e '.error' > /dev/null; then
        local ERROR_MSG
        ERROR_MSG=$(echo "$api_response" | jq -r '.error')
        gum style --foreground 1 "✖ API Error: $ERROR_MSG"
        exit 1
    fi

    echo "$api_response"
}

# Parses the API JSON response, validates the contents, and exports them as global variables.
# Exits with an error if data is incomplete.
# Usage: parse_and_export_docker_vars "<json_string>"
parse_and_export_docker_vars() {
    local api_response="$1"

    export DOCKER_REGISTRY=$(echo "$api_response" | jq -r '.container_registry')
    export DOCKER_IMAGE=$(echo "$api_response" | jq -r '.image_name')
    export DOCKER_USER=$(echo "$api_response" | jq -r '.username')
    export DOCKER_TOKEN=$(echo "$api_response" | jq -r '.password')

    if [ -z "$DOCKER_REGISTRY" ] || [ "$DOCKER_REGISTRY" == "null" ] || \
       [ -z "$DOCKER_USER" ] || [ "$DOCKER_USER" == "null" ] || \
       [ -z "$DOCKER_TOKEN" ] || [ "$DOCKER_TOKEN" == "null" ] || \
       [ -z "$DOCKER_IMAGE" ] || [ "$DOCKER_IMAGE" == "null" ]; then

        gum style --foreground 1 "✖ Error: The license API returned incomplete or null data."
        gum style --faint "This can happen with an invalid license key. Please check the response below."
        echo "$api_response" | jq .
        exit 1
    fi
}

# Performs the Docker login, pull, and run sequence.
# Uses the globally exported Docker variables.
# Usage: deploy_container
deploy_container() {
    export FULL_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_IMAGE}"

    run_with_spinner "Logging into Docker registry..." \
        "echo \"$DOCKER_TOKEN\" | docker login \"$DOCKER_REGISTRY\" -u \"$DOCKER_USER\" --password-stdin" || exit 1

    gum style --bold --foreground 212 "Pulling EmbodiedOS container image..."
    gum style --faint "This may take several minutes depending on your network connection."
    docker pull "$FULL_IMAGE_NAME" || {
        gum style --foreground 1 "✖ Error: Failed to pull Docker image."
        exit 1
    }
    gum style --foreground 2 "✔ Success: Pulled latest image."

    run_with_spinner "Starting EmbodiedOS container..." \
        "$DOCKER_RUN_CMD" || exit 1
}

do_install() {
    local license_key="$1"
    if [ -z "$license_key" ]; then
        gum style --foreground 1 "✖ Error: A license key is required."
        echo "Usage: emos install <your_license_key>"
        exit 1
    fi

    # Check for existing installation and confirm overwrite
    if docker inspect "$CONTAINER_NAME" &> /dev/null; then
        gum style --foreground 3 --padding "1 2" --border double --border-foreground 3 \
            "[!] An existing EmbodiedOS container was found."
        if ! gum confirm "This will REMOVE the existing container and perform a fresh installation. Are you sure?"; then
            gum style --foreground 1 "✖ Installation aborted by user."
            exit 1
        fi
    fi

    display_art
    echo "🚀 Starting EmbodiedOS installation..."
    mkdir -p "$CONFIG_DIR"

    local API_RESPONSE
    API_RESPONSE=$(api_validate_license "$license_key")
    gum style --foreground 2 "✔ License key validated successfully."

    # Remove old container now that license is confirmed valid
    if docker inspect "$CONTAINER_NAME" &> /dev/null; then
        gum style --bold --foreground 212 "Proceeding with re-installation. Removing existing container..."
        run_with_spinner "Stopping the current container..." "docker stop \"$CONTAINER_NAME\""
        run_with_spinner "Removing the old container..." "docker rm \"$CONTAINER_NAME\""
    fi

    parse_and_export_docker_vars "$API_RESPONSE"
    echo "$license_key" > "$LICENSE_FILE" # Save the license key
    deploy_container

    # --- install: systemd service ---
    gum style --bold --foreground 212 "Creating systemd service..."
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
    if gum confirm "Create/overwrite systemd service file for auto-restart?"; then
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
        gum style --bold "Please run 'emos update' again to update your EmbodiedOS container."
        exit 0
    elif [ "$CLI_UPDATE_STATUS" -ne 0 ]; then
        gum style --foreground 1 "✖ Error: The CLI update check failed. Cannot proceed."
        exit 1
    fi
    gum style --foreground 2 "✔ Your emos CLI is up to date."
    echo

    # Check for license file before proceeding
    if [ ! -f "$LICENSE_FILE" ]; then
        gum style --foreground 1 "✖ Error: No existing installation found."
        echo "Please run 'emos install <license_key>' first."
        exit 1
    fi
    echo "🚀 Checking for EmbodiedOS container updates..."
    LICENSE_KEY=$(cat "$LICENSE_FILE")

    local API_RESPONSE
    API_RESPONSE=$(api_validate_license "$LICENSE_KEY")
    gum style --foreground 2 "✔ License verified."

    parse_and_export_docker_vars "$API_RESPONSE"

    # Stop and remove the old container before deploying the new one
    run_with_spinner "Stopping the current container..." "docker stop \"$CONTAINER_NAME\""
    run_with_spinner "Removing the old container..." "docker rm \"$CONTAINER_NAME\""

    deploy_container

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
            display_art
            echo "EMOS is a self-contained automation layer for your robot."
            echo "This tool helps you manage its lifecycle on this machine."
            echo "Developed with ❤️  by Automatika Robotics."
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
