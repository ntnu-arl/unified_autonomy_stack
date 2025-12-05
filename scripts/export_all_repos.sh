#!/bin/bash

# Script to export the repos in each workspace to .repos files in the repos/ directory using vcstool
UAS_REPO_ROOT=$(dirname "$0")/..
REPOS_DIR="$UAS_REPO_ROOT/repos"
WORKSPACES_DIR="$UAS_REPO_ROOT/workspaces"

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

# Check if the workspaces directory exists
if [ ! -d "$WORKSPACES_DIR" ]; then
    echo -e "${RED}Error: $WORKSPACES_DIR directory not found!${NC}"
    exit 1
fi

# Iterate over each workspace directory
for workspace in "$WORKSPACES_DIR"/ws_*; do
    echo "workspace: $workspace"
    if [ -d "$workspace" ]; then
        workspace_name=$(basename "$workspace")
        echo "workspace name: $workspace_name"

        repos_file="$REPOS_DIR/${workspace_name}.repos"
        echo -e "${YELLOW}Exporting workspace: $workspace_name to $repos_file${NC}"
        if vcs export  "$workspace" > "$repos_file"; then
            echo -e "${GREEN}✓ Successfully exported to $repos_file${NC}"
        else
            echo -e "${RED}✗ Failed to export $workspace_name${NC}"
            exit 1
        fi

        exact_repos_file="$REPOS_DIR/${workspace_name}_exact.repos"
        echo -e "${YELLOW}Exporting workspace: $workspace_name to $exact_repos_file${NC}"
        if vcs export --exact-with-tags "$workspace" > "$exact_repos_file"; then
            echo -e "${GREEN}✓ Successfully exported to $exact_repos_file${NC}"
        else
            echo -e "${RED}✗ Failed to export $workspace_name${NC}"
            exit 1
        fi

        echo ""
    fi
done


# Export robot_bringup
repos_file="$REPOS_DIR/robot_bringup.repos"
echo -e "${YELLOW}Exporting robot_bringup.repos"
if vcs export  "$WORKSPACES_DIR/robot_bringup" > "$repos_file"; then
    echo -e "${GREEN}✓ Successfully exported to $repos_file${NC}"
else
    echo -e "${RED}✗ Failed to export $workspace_name${NC}"
    exit 1
fi

exact_repos_file="$REPOS_DIR/robot_bringup_exact.repos"
echo -e "${YELLOW}Exporting robot_bringup_exact.repos"
if vcs export --exact-with-tags "$WORKSPACES_DIR/robot_bringup" > "$exact_repos_file"; then
    echo -e "${GREEN}✓ Successfully exported to $exact_repos_file${NC}"
else
    echo -e "${RED}✗ Failed to export robot_bringup${NC}"
    exit 1
fi
echo -e "${GREEN}All workspaces exported successfully!${NC}"
