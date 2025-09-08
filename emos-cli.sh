#!/bin/bash

# ==============================================================================
# emos - EmbodiedOS Management CLI v0.1.2
# ==============================================================================

# --- Configuration ---
EMOS_VERSION="0.1.2"
CONFIG_DIR="$HOME/.config/emos"
RECIPES_DIR="$HOME/emos/recipes"
LICENSE_FILE="$CONFIG_DIR/license.key"
CONTAINER_NAME="emos"
SERVICE_NAME="emos.service"
DOCKER_RUN_CMD="docker run -d -it --restart always --privileged -v /dev/bus/usb:/dev/bus/usb -v $HOME/emos:/emos --name \"\$CONTAINER_NAME\" --network host --runtime nvidia --gpus=all \"\$FULL_IMAGE_NAME\""
INSTALLER_URL="https://raw.githubusercontent.com/automatika-robotics/emos-cli/main/install.sh"

# --- Support API Endpoints ---
API_BASE_URL="https://support-api.automatikarobotics.com/api"
CREDENTIALS_ENDPOINT="$API_BASE_URL/registrations/ghcr-credentials"
RECIPES_LIST_ENDPOINT="$API_BASE_URL/recipes"
RECIPE_PULL_ENDPOINT="$API_BASE_URL/recipes/%s" # %s is a placeholder for the filename

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
    choice=$(gum choose --height 9 --cursor-prefix "➜ " --header " " \
        --item.foreground $THEME_NEUTRAL \
        --cursor.foreground $THEME_RED \
        "install   - Install and start EMOS using a license key." \
        "update    - Update the CLI and/or the EMOS container." \
        "ls        - List available automation recipes." \
        "run       - Execute a specific automation recipe." \
        "recipes   - List available recipes for download." \
        "pull      - Download and install a specific recipe." \
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
            "ls")
                gum style --faint "Voilà ! Copy and press Enter."
                printf "emos ls\n"
                ;;
            "run")
                gum style --faint "Voilà ! Copy, add recipe name and press Enter."
                printf "emos run <recipe_name>\n"
                ;;
            "recipes")
                gum style --faint "Voilà ! Copy and press Enter."
                printf "emos recipes\n"
                ;;
            "pull")
                gum style --faint "Voilà ! Copy, add recipe name and press Enter."
                printf "emos pull <recipe_name>\n"
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

# Lists available automation recipes found in the recipes directory.
do_list_recipes() {
    display_art

    # Check if the recipes directory exists and is not empty
    if [ ! -d "$RECIPES_DIR" ] || ! ls -d "$RECIPES_DIR"/*/ >/dev/null 2>&1; then
        gum style --foreground 3 "No recipes found in '$RECIPES_DIR'."
        return
    fi

    # Prepare the header for the table
    local table_data="NAME,DESCRIPTION"

    # Loop through each recipe's subdirectory
    for recipe_path in "$RECIPES_DIR"/*/; do
        local short_name
        short_name=$(basename "$recipe_path")

        local manifest_file="${recipe_path}manifest.json"
        local long_name="-" # Default value if manifest or name is missing

        if [ -f "$manifest_file" ]; then
            # Use jq to extract the 'name' field, ensuring it's not null or empty
            local name_from_json
            name_from_json=$(jq -r '.name' "$manifest_file")
            if [ -n "$name_from_json" ] && [ "$name_from_json" != "null" ]; then
                long_name="$name_from_json"
            fi
        fi

        # Append the new row to our CSV data, quoting the full name
        table_data+="\n$short_name,\"$long_name\""
    done

    # Pipe the CSV data into gum table for a pretty output
    echo -e "$table_data" | gum table --selected.foreground "$THEME_RED"

}

# Runs a specific automation recipe.
# Usage:do_run_recipe <recipe_name>
do_run_recipe() {
    local recipe_name="$1"

    # Check if a recipe name was provided
    if [ -z "$recipe_name" ]; then
        gum style --foreground 1 "✖ Error: A recipe name is required."
        echo "Usage: emos run <recipe_name>"
        gum style --faint "Run 'emos ls' to see a list of available recipes."
        exit 1
    fi

    # Check if the recipe_run.sh script exists in the user's PATH
    if ! command -v recipe_run.sh &> /dev/null; then
        gum style --foreground 1 "✖ Error: The 'recipe_run.sh' script is not found in your PATH."
        gum style --faint "Please ensure the EmbodiedOS execution environment is correctly installed."
        exit 1
    fi

    # Check if the recipe directory actually exists
    local recipe_path="$RECIPES_DIR/$recipe_name"
    if [ ! -d "$recipe_path" ]; then
        gum style --foreground 1 "✖ Error: Recipe '$recipe_name' not found in '$RECIPES_DIR'."
        gum style --faint "Please check the spelling. Run 'emos ls' to see available recipes."
        exit 1
    fi

    # If all checks pass, execute the recipe
    display_art
    gum style --bold --foreground "$THEME_RED" --padding "0 1" "🚀 Launching recipe: $recipe_name"
    echo

    # Execute the run script directly, allowing the user to see its output and interact.
    recipe_run.sh --recipe_name="$recipe_name"

    local exit_code=$?
    echo
    if [ $exit_code -eq 0 ]; then
        gum style --foreground 2 "✔ Recipe '$recipe_name' finished successfully."
    else
        gum style --foreground 1 "✖ Recipe '$recipe_name' exited with an error (code: $exit_code)."
    fi
}

# Lists available recipes from the remote API.
do_list_remote_recipes() {
    display_art

    local TMP_FILE
    TMP_FILE=$(mktemp)

    # Fetch the list of recipes from the API
    if ! gum spin --spinner dot --title "Fetching available recipes from server..." -- \
        curl -sSLf "$RECIPES_LIST_ENDPOINT" -o "$TMP_FILE"; then
        rm "$TMP_FILE"
        gum style --foreground 1 "✖ Error: Could not connect to the recipes API."
        exit 1
    fi

    local API_RESPONSE
    API_RESPONSE=$(cat "$TMP_FILE")
    rm "$TMP_FILE"

    if [ -z "$API_RESPONSE" ] || ! echo "$API_RESPONSE" | jq empty; then
        gum style --foreground 1 "✖ Error: Recipes API returned an invalid or empty response."
        exit 1
    fi

    # Use jq to parse the JSON array and format it as CSV for gum table
    # It extracts the filename, removes the .zip, and gets the full name.
    local table_data="RECIPE NAME,DESCRIPTION"
    table_data+=$(echo "$API_RESPONSE" | jq -r '.[] | "\n\(.filename | sub(".zip$"; "")),\"\(.name)\""')

    gum style --bold --foreground "$THEME_BLUE" "Available Recipes for Download"
    echo -e "$table_data" | gum table --selected.foreground "$THEME_RED"

    gum style --faint "Use 'emos pull <recipe_name>' to install a recipe."
}

# Downloads and unzips a specific recipe.
# Usage: do_pull_recipe <recipe_short_name>
do_pull_recipe() {
    local recipe_name="$1"

    if [ -z "$recipe_name" ]; then
        gum style --foreground 1 "✖ Error: A recipe name is required."
        echo "Usage: emos pull <recipe_name>"
        gum style --faint "Run 'emos recipes' to see a list of available recipes."
        exit 1
    fi

    # Reconstruct the filename and the download URL
    local zip_filename="${recipe_name}.zip"
    local download_url
    download_url=$(printf "$RECIPE_PULL_ENDPOINT" "$zip_filename")

    # Ensure the destination directory exists
    mkdir -p "$RECIPES_DIR"
    local temp_zip_path="/tmp/${zip_filename}"

    display_art
    gum style --bold --foreground "$THEME_BLUE" "Installing recipe: $recipe_name"

    # Download the recipe zip file
    run_with_spinner "Downloading from $download_url..." \
        "curl -sSLf \"$download_url\" -o \"$temp_zip_path\"" || exit 1

    # Unzip the file into the recipes directory
    # The -o flag overwrites existing files without prompting
    run_with_spinner "Unzipping recipe to $RECIPES_DIR..." \
        "unzip -o \"$temp_zip_path\" -d \"$RECIPES_DIR\"" || exit 1

    # Clean up the downloaded zip file
    rm "$temp_zip_path"

    gum style --border double --padding "1 5" --border-foreground 2 "✔ Recipe '$recipe_name' installed successfully!"
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
            "$CREDENTIALS_ENDPOINT" -o "$TMP_FILE"; then
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

    gum style --bold --foreground "$THEME_BLUE" "Pulling EmbodiedOS container image..."
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
        gum style --bold --foreground "$THEME_BLUE" "Proceeding with re-installation. Removing existing container..."
        run_with_spinner "Stopping the current container..." "docker stop \"$CONTAINER_NAME\""
        run_with_spinner "Removing the old container..." "docker rm \"$CONTAINER_NAME\""
    fi

    parse_and_export_docker_vars "$API_RESPONSE"
    echo "$license_key" > "$LICENSE_FILE" # Save the license key

    # Create emos content dir
    gum style --faint "Ensuring local directory exists at $HOME/emos..."
    mkdir -p "$HOME/emos"

    deploy_container

    # --- install: systemd service ---
    gum style --bold --foreground "$THEME_BLUE" "Creating systemd service..."
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
        gum style --foreground "$THEME_BLUE" "💡 The emos CLI has been updated to the latest version."
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
        ls)
            do_list_recipes
            ;;
        run)
            do_run_recipe "$2"
            ;;
        recipes)
            do_list_remote_recipes
            ;;
        pull)
            do_pull_recipe "$2"
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
