#!/bin/bash

# Script to import all .repos files from repos/ directory using vcstool
UAS_REPO_ROOT=$(dirname "$0")/..
REPOS_DIR="$UAS_REPO_ROOT/repos"

# Parse command line arguments
USE_EXACT=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --exact) USE_EXACT=true ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Import robot bringup
echo "$REPOS_DIR/robot_bringup.repos"
if [ "$USE_EXACT" = true ]; then
    robot_bringup_repos_file="$REPOS_DIR/robot_bringup_exact.repos"
else
    robot_bringup_repos_file="$REPOS_DIR/robot_bringup.repos"
fi
if vcs import --recursive "$UAS_REPO_ROOT/workspaces" < "$robot_bringup_repos_file"; then
    echo -e "${GREEN}✓ Successfully imported $repos_file${NC}"
else
    echo -e "${RED}✗ Failed to import $repos_file${NC}"
    exit 1
fi

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if repos directory exists
if [ ! -d "$REPOS_DIR" ]; then
    echo -e "${RED}Error: $REPOS_DIR directory not found!${NC}"
    exit 1
fi

# Find all .repos files in the repos directory
if [ "$USE_EXACT" = true ]; then
    repos_files=$(find "$REPOS_DIR" -name "ws*exact.repos" -type f | sort)
    suffix="_exact.repos"
else
    repos_files=$(find "$REPOS_DIR" -name "ws*.repos" ! -name "*exact.repos" -type f | sort)
    suffix=".repos"
fi

# Check if any .repos files were found
if [ -z "$repos_files" ]; then
    echo -e "${YELLOW}Warning: No$([ "$USE_EXACT" = true ] && echo " exact") .repos files found in $REPOS_DIR directory${NC}"
    exit 0
fi

# Display found files
echo -e "${GREEN}Found the following .repos files:${NC}"
echo "$repos_files"
echo ""

# Import each repos file
echo -e "${GREEN}Starting import...${NC}"
for repos_file in $repos_files; do
    WS="$UAS_REPO_ROOT/workspaces/$(basename "$repos_file" "$suffix")"
    mkdir -p "$WS"
    echo -e "${RED} Creating workspace: $WS ${NC}"
    echo -e "${YELLOW}Importing from: $repos_file${NC}"
    if vcs import --recursive "$WS" < "$repos_file"; then
        echo -e "${GREEN}✓ Successfully imported $repos_file${NC}"
    else
        echo -e "${RED}✗ Failed to import $repos_file${NC}"
        exit 1
    fi
    echo ""
done

echo -e "${GREEN}All repos files imported successfully!${NC}"

# Apply mmwave_ti_ros patch if it exists
MMWAVE_REPOS="$REPOS_DIR/ws_mmwave_ti_ros.repos"
MMWAVE_WS="$UAS_REPO_ROOT/workspaces/ws_mmwave_ti_ros"
MMWAVE_SRC="$MMWAVE_WS/src/mmwave_ti_ros"
MMWAVE_PATCH="$REPOS_DIR/patches/ws_mmwave_ti_ros.patch"

if [ -f "$MMWAVE_REPOS" ] && [ -f "$MMWAVE_PATCH" ] && [ -d "$MMWAVE_SRC" ]; then
    echo -e "${YELLOW}Applying mmwave_ti_ros patch...${NC}"
    # Convert patch path to absolute
    ABS_MMWAVE_PATCH="$(cd "$(dirname "$MMWAVE_PATCH")" && pwd)/$(basename "$MMWAVE_PATCH")"
    if (cd "$MMWAVE_SRC" && git apply "$ABS_MMWAVE_PATCH"); then
        echo -e "${GREEN}✓ Successfully applied mmwave_ti_ros patch${NC}"
    else
        echo -e "${RED}✗ Failed to apply mmwave_ti_ros patch${NC}"
        exit 1
    fi
fi