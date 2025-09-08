#!/bin/bash
set -e

# --- EMOS Recipe Runner ---
# A simple script to execute a specific recipe.
# This script is called by 'emos run <recipe_name>'.

# --- Argument Parsing ---
RECIPE_NAME=""
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --recipe_name)
            if [ -n "$2" ]; then
                RECIPE_NAME="$2"
                shift 2
            else
                echo "Error: --recipe_name requires a non-empty argument." >&2
                exit 1
            fi
            ;;
        *)
            echo "Unknown parameter passed: $1" >&2
            exit 1
            ;;
    esac
done

# --- Validation ---
if [ -z "$RECIPE_NAME" ]; then
    echo "Usage: $0 --recipe_name <name>" >&2
    exit 1
fi

# --- Stub Execution Logic ---
# In the future, this is where the logic to start the
# recipe's Docker Compose file, etc., will go.

echo
echo "🚀 Preparing to run recipe: '$RECIPE_NAME'..."
echo "(This is a stub. Full execution logic will be added later.)"
echo

