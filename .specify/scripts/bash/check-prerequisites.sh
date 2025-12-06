#!/bin/bash
# A simple script to check for the existence of a feature spec file.
# This is a simplified version for demonstration purposes.

json_output=false
paths_only=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --json) json_output=true ;;
        --paths-only) paths_only=true ;;
    esac
    shift
done

branch_name=$(git rev-parse --abbrev-ref HEAD)
feature_dir="specs/$branch_name"
feature_spec="$feature_dir/spec.md"

if [ ! -f "$feature_spec" ]; then
    echo "Error: Spec file not found at $feature_spec"
    exit 1
fi

if [ "$json_output" = true ] && [ "$paths_only" = true ]; then
    echo "{\"FEATURE_DIR\":\"$feature_dir\",\"FEATURE_SPEC\":\"$feature_spec\"}"
fi

