#!/bin/bash
set -e

# =========================
# EMOS - MAPPING RUNNER
# =========================

# --- Theme ---
THEME_RED="#d54e53"
THEME_BLUE="#81a2be"
THEME_NEUTRAL="#EEF2F3"

# A wrapper for gum logging functions to standardize output format
# Usage: log "Your message here"
log() {
    gum style --foreground 240 "│"
    gum style --foreground 250 "├─ $1"
}

success() {
    gum style --foreground 2 "│"
    gum style --foreground 2 "├─ ✔ Success: $1"
}

warn() {
    gum style --foreground 3 "│"
    gum style --foreground 3 "├─ ! Warning: $1"
}

error() {
    gum style --foreground 1 "│"
    gum style --foreground 1 "├─ ✖ Error: $1"
    exit 1
}

# A wrapper for gum spin to show a loader for long-running commands
# Usage: run_with_spinner "Doing a thing..." "my_command --with --args"
run_with_spinner() {
  local title="$1"
  local cmd="$2"
  local tmpfile
  tmpfile=$(mktemp)

  gum spin --spinner dot --title "$title" -- bash -c "$cmd >$tmpfile 2>&1"
  local EXIT_CODE=$?

  if [ $EXIT_CODE -eq 0 ]; then
    success "$title"
  else
    error "$title"
    gum style --faint "  Command failed with output:"
    gum format -- "$(cat "$tmpfile")"
  fi

  rm -f "$tmpfile"
  return $EXIT_CODE
}


print_header() {
    gum style --bold --padding "1 2" --border thick --border-foreground "$THEME_BLUE" --foreground "$THEME_BLUE" "$1"
}

# --- Argument Parsing ---
EMOS_ROOT="/emos"
RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
MAP_NAME=$(gum input --cursor.foreground $THEME_RED --placeholder "Enter a map name (default: map_default)")
MAP_NAME=${MAP_NAME:-map_default}   # fallback to default if empty

# --- Configuration & Pre-flight Checks ---
print_header "EMOS - PRE-MAPPING SETUP"

CONTAINER_NAME="emos"
LOG_DIR="${LOG_DIR:-$HOME/emos/logs}"
DATETIME=$(date +"%Y%m%d_%H%M%S")
MAPPING_LAUNCH_FILE="$EMOS_ROOT/robot/launch/bringup_mapping.py"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/mapping_${MAP_NAME}_${DATETIME}.log}"

# --- Load Topics from mapping manifest ---
MAPPING_JSON_FILE="$HOME/emos/robot/mapping.json"

if [[ ! -f "$MAPPING_JSON_FILE" ]]; then
    error "Mapping manifest file not found. Run 'emos update' to get the latest files."
fi

# Use jq to read array into Bash array
mapfile -t TOPICS < <(jq -r '.topics[]' "$MAPPING_JSON_FILE")


log "Map Name: $MAP_NAME"
log "Running Mapping Using RMW Implementation: $RMW_IMPLEMENTATION"
log "Container Name: $CONTAINER_NAME"

# --- Host & Container Management ---
print_header "HOST & CONTAINER MANAGEMENT"

log "Killing all ROS processes on host (sudo may prompt for password)..."
sudo pkill -f roslaunch >/dev/null 2>&1 || true
sudo pkill -f roscore >/dev/null 2>&1 || true
sudo pkill -f ros2 >/dev/null 2>&1 || true
sleep 1
success "Terminated host ROS processes."

if [ -z "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    error "Container '$CONTAINER_NAME' does not exist! Run EMOS Setup first."
fi

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    run_with_spinner "Stopping existing EMOS container..." "docker stop $CONTAINER_NAME >/dev/null 2>&1" || true
fi

run_with_spinner "Starting EMOS container..." "docker start $CONTAINER_NAME >/dev/null 2>&1" || error "Failed to start container."


# --- RMW Configuration ---
print_header "RMW CONFIGURATION"

# Set RMW_IMPLEMENTATION in ros_entrypoint.sh
log "Setting RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION inside container $CONTAINER_NAME..."

docker exec "$CONTAINER_NAME" bash -c "
  if grep -q '^export RMW_IMPLEMENTATION=' /ros_entrypoint.sh; then
    # Replace existing line
    sed -i 's|^export RMW_IMPLEMENTATION=.*|export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION|' /ros_entrypoint.sh
  else
    # Insert after shebang
    sed -i '1a export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION' /ros_entrypoint.sh
  fi
"

# --- Hardware & Sensor Launch ---
print_header "HARDWARE & SENSOR LAUNCH"

mkdir -p "${LOG_DIR}"
log "All output will be saved to: ${LOG_FILE}"

run_with_spinner "Launching mapping hardware..." \
    "docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ros2 launch $MAPPING_LAUNCH_FILE'"


# --- Node Verification ---
print_header "VERIFYING ROS2 NODES"
sleep 5 # Give nodes a moment to initialize

ROBOT_JSON_FILE="$HOME/$EMOS_ROOT/robot/manifest.json"
mapfile -t EXPECTED_NODES < <(jq -r '.base[]' "$ROBOT_JSON_FILE")

ALL_PRESENT=true
for NODE in "${EXPECTED_NODES[@]}"; do
    NODE_CLEAN=$(echo "$NODE" | xargs)

    # Use gum spin for the verification loop
    if ! gum spin --spinner dot --title "Checking for node '$NODE_CLEAN'..." -- \
        bash -c "for i in {1..10}; do docker exec $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ros2 node list' | grep -Fq \"$NODE_CLEAN\" && exit 0; sleep 1; done; exit 1"; then
        error "Node '$NODE_CLEAN' did not appear within 10 seconds!"
        ALL_PRESENT=false
    else
        success "Node '$NODE_CLEAN' is active."
    fi
done

if [[ "$ALL_PRESENT" == "false" ]]; then
  error "Required nodes are missing. Stopping container."
  docker stop $CONTAINER_NAME >/dev/null 2>&1
  exit 1
fi

# --- Final Configuration ---
print_header "FINAL CONFIGURATION"

run_with_spinner "Deactivating autonomous mode..." \
"docker exec -d $CONTAINER_NAME bash -c 'source ros_entrypoint.sh && ./$EMOS_ROOT/robot/scripts/deactivate_autonomous_mode.sh'"

# --- Recipe Execution ---
print_header "LAUNCHING DATA RECORDING"

BAG_DIR="${EMOS_ROOT}/maps"
BAG_PATH="${BAG_DIR}/${MAP_NAME}"
OUTPUT_DIR="$HOME/emos/maps/${MAP_NAME}"

if gum confirm --prompt.foreground $THEME_BLUE --selected.background $THEME_RED "Get your robot ready and confirm to start mapping data recording"; then
    log "Starting data recording and saving to ${OUTPUT_DIR}..."
else
    warn "User canceled mapping."
    exit 0
fi

# 1. Ensure BAG_DIR exists in container
docker exec ${CONTAINER_NAME} bash -c "mkdir -p ${BAG_DIR}"

# 2. Check if bag already exists in container
if docker exec ${CONTAINER_NAME} bash -c "[ -f '${BAG_PATH}.tar.gz' ]"; then
    warn "A map data file already exists at ${OUTPUT_DIR}.tar.gz"

    if gum confirm --prompt.foreground $THEME_BLUE --selected.background $THEME_RED "Do you want to overwrite it?"; then
        log "Overwriting existing map data at ${OUTPUT_DIR}.tar.gz"
        docker exec ${CONTAINER_NAME} bash -c "rm -f '${BAG_PATH}.tar.gz'"
    else
        error "User chose not to overwrite existing map data. Aborting."
    fi
fi

# 3. Start ros2 bag recording with nice defaults
docker exec ${CONTAINER_NAME} bash -c "source ros_entrypoint.sh && \
    ros2 bag record -o '${BAG_PATH}' \
    --storage mcap \
    --compression-mode file \
    --compression-format zstd \
	--topics ${TOPICS[*]}" &
BAG_PID=$!

cleanup() {
    run_with_spinner "Got Ctrl+C, terminating mapping..." \
    "kill $BAG_PID && docker restart ${CONTAINER_NAME} && sleep 5"
    run_with_spinner "Zipping & Saving Mapped Data..." \
    "docker exec ${CONTAINER_NAME} bash -c 'tar -czvf ${BAG_PATH}.tar.gz -C ${BAG_DIR} ${MAP_NAME} && rm -rf ${BAG_PATH}'"
    success "Map data saved to ${OUTPUT_DIR}.tar.gz"
    exit 0
}

log "Map Data Recording Started ..."
log "Press Ctrl+C once to end data recording process."

# Trap Ctrl+C (SIGINT) and call cleanup
trap cleanup SIGINT

# Keep script alive until rosbag stops
wait $BAG_PID
