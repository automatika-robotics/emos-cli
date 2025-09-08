#!/bin/bash
set -e

# =========================
# EMOS - RECIPE RUNNER
# =========================

# --- Theme ---
THEME_RED="#d54e53"
THEME_BLUE="#81a2be"
THEME_NEUTRAL="#EEF2F3"

# A wrapper for gum spin to show a loader for long-running commands
# Usage: run_with_spinner "Doing a thing..." "my_command --with --args"
run_with_spinner() {
    local title="$1"
    local cmd="$2"
    local OUTPUT
    local EXIT_CODE

    # `2>&1` redirects stderr to stdout, so we capture everything.
    OUTPUT=$(eval "$cmd" 2>&1)
    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        # On success, show a brief spinner animation for a polished feel.
        gum spin --spinner dot --title "$title" -- sleep 1
        gum style --foreground 2 "✔ Success:" "$title"
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

log() {
    gum style --foreground 240 "│"
    gum style --foreground 250 "├─ $1"
}

warn() {
    gum style --foreground 3 "│"
    gum style --foreground 3 "├─ WARNING: $1"
}

error() {
    gum style --foreground 1 "│"
    gum style --foreground 1 "├─ ERROR: $1"
    exit 1
}

print_header() {
    gum style --bold --padding "1 2" --border thick --border-foreground "$THEME_BLUE" --foreground "$THEME_BLUE" "$1"
}

# --- Argument Parsing ---
EMOS_ROOT="/emos"
RECIPES_ROOT="/emos/recipes"
RECIPES_DIR="$HOME/emos/recipes"
RECIPE_NAME=""

for arg in "$@"; do
  case $arg in
    --recipe_name=*)
      RECIPE_NAME="${arg#*=}"
      ;;
    *)
      error "Unknown argument: $arg"
      ;;
  esac
done

if [ -z "$RECIPE_NAME" ]; then
    error "Recipe name was not provided. Usage: $0 --recipe_name=<name>"
fi

# --- Configuration & Pre-flight Checks ---
print_header "EMOS - PRE-RECIPE SETUP"

CONTAINER_NAME="emos"
MANIFEST_FILE="${RECIPES_DIR}/${RECIPE_NAME}/manifest.json"
LOG_DIR="${LOG_DIR:-$HOME/emos/logs}"
DATETIME=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_FILE:-${LOG_DIR}/${RECIPE_NAME}_${DATETIME}.log}"

log "Recipe Name: $RECIPE_NAME"
log "Container Name: $CONTAINER_NAME"
log "Manifest File: $MANIFEST_FILE"

if [[ ! -f "${MANIFEST_FILE}" ]]; then
    error "Manifest file not found at ${MANIFEST_FILE}"
fi

# Extract manifest data
ACTIVATE_AUTONOMOUS_MODE=$(jq -r '.autonomous_mode' "${MANIFEST_FILE}" 2>/dev/null || echo "false")
RUN_WEB_CLIENT=$(jq -r '.web_client' "${MANIFEST_FILE}" 2>/dev/null || echo "false")
readarray -t ROBOT_SENSORS < <(jq -r '.sensors[]' "${MANIFEST_FILE}" 2>/dev/null)

log "Required sensors: ${ROBOT_SENSORS[*]}"
log "Autonomous mode required: $ACTIVATE_AUTONOMOUS_MODE"
log "Web client required: $RUN_WEB_CLIENT"

# --- Host & Container Management ---
print_header "HOST & CONTAINER MANAGEMENT"

log "Killing all ROS processes on host (sudo may prompt for password)..."
sudo pkill -f roslaunch >/dev/null 2>&1 || true
sudo pkill -f roscore >/dev/null 2>&1 || true
sudo pkill -f ros2 >/dev/null 2>&1 || true
sleep 1
gum style --foreground 2 "✔ Success: Terminated host ROS processes."

if [ -z "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    error "Container '$CONTAINER_NAME' does not exist! Run EMOS Setup first."
fi

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    run_with_spinner "Stopping existing EMOS container..." "docker stop $CONTAINER_NAME >/dev/null 2>&1" || true
fi

run_with_spinner "Starting EMOS container..." "docker start $CONTAINER_NAME >/dev/null 2>&1" || error "Failed to start container."

# --- Hardware & Sensor Launch ---
print_header "HARDWARE & SENSOR LAUNCH"

run_with_spinner "Launching robot base hardware..." \
    "docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ros2 launch $EMOS_ROOT/robot/launch/bringup_robot.py'"

for sensor in "${ROBOT_SENSORS[@]}"; do
    run_with_spinner "Launching sensor: $sensor..." \
        "docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ros2 launch $EMOS_ROOT/robot/launch/bringup_${sensor}.py config_file:=$RECIPES_ROOT/$RECIPE_NAME/${sensor}_config.yaml'"
done

# --- Node Verification ---
print_header "VERIFYING ROS2 NODES"
sleep 5 # Give nodes a moment to initialize

ROBOT_JSON_FILE="$HOME/$EMOS_ROOT/robot/manifest.json"
mapfile -t EXPECTED_NODES < <(jq -r '.base[]' "$ROBOT_JSON_FILE")

for sensor in "${ROBOT_SENSORS[@]}"; do
  node=$(jq -r --arg s "$sensor" 'select(has($s))[$s] // empty' "$ROBOT_JSON_FILE")
  if [[ -n "$node" ]]; then
    EXPECTED_NODES+=("$node")
  fi
done

log "Verifying required ROS2 nodes are active..."
ALL_PRESENT=true
for NODE in "${EXPECTED_NODES[@]}"; do
    NODE_CLEAN=$(echo "$NODE" | xargs)
    
    # Use gum spin for the verification loop
    if ! gum spin --spinner dot --title "Checking for node '$NODE_CLEAN'..." -- \
        bash -c "for i in {1..10}; do docker exec $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ros2 node list' | grep -Fq \"$NODE_CLEAN\" && exit 0; sleep 1; done; exit 1"; then
        error "Node '$NODE_CLEAN' did not appear within 10 seconds!"
        ALL_PRESENT=false
    else
        gum style --foreground 2 "✔ Success: Node '$NODE_CLEAN' is active."
    fi
done

if [[ "$ALL_PRESENT" == "false" ]]; then
  error "Required nodes are missing. Stopping container."
  docker stop $CONTAINER_NAME >/dev/null 2>&1
  exit 1
fi

# --- Final Configuration ---
print_header "FINAL CONFIGURATION"

if [[ "$ACTIVATE_AUTONOMOUS_MODE" == "true" ]]; then
  run_with_spinner "Activating autonomous mode..." \
    "docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ./$EMOS_ROOT/robot/scripts/activate_autonomous_mode.sh'"
  warn "ATTENTION: Autonomous Mode is now ON."
else
  run_with_spinner "Deactivating autonomous mode..." \
    "docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ./$EMOS_ROOT/robot/scripts/deactivate_autonomous_mode.sh'"
fi

# --- Recipe Execution ---
print_header "LAUNCHING RECIPE: $RECIPE_NAME"

mkdir -p "${LOG_DIR}"
log "All output will be saved to: ${LOG_FILE}"

if [[ "${RUN_WEB_CLIENT}" == "true" ]]; then
    run_with_spinner "Starting web client in background..." \
        "docker exec -d ${CONTAINER_NAME} bash -c 'source ros_entrypoint.sh && ros2 run automatika_embodied_agents tiny_web_client'"
    log "Web client should be available at http://<ROBOT_IP>:8080"
fi

gum style --bold --padding "1 0" --foreground 2 "BEGIN RECIPE OUTPUT"

# Execute the main recipe, teeing output to the log file.
docker exec -it "${CONTAINER_NAME}" bash -c \
    "source ros_entrypoint.sh && python3 ${RECIPES_ROOT}/${RECIPE_NAME}/recipe.py" \
    | tee "${LOG_FILE}"

RECIPE_EXIT_CODE=${PIPESTATUS[0]}

gum style --bold --padding "1 0" --foreground 2 "END RECIPE OUTPUT"

if [ $RECIPE_EXIT_CODE -eq 0 ]; then
    gum style --foreground 2 "✔ Recipe '$RECIPE_NAME' finished successfully."
else
    error "Recipe '$RECIPE_NAME' exited with an error (code: $RECIPE_EXIT_CODE)."
fi


