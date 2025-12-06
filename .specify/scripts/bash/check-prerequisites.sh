#!/bin/bash
# A simple script to check for the existence of a feature spec file.
# This is a simplified version for demonstration purposes.

json_output=false
paths_only=false
require_tasks=false
include_tasks=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --json) json_output=true ;;
        --paths-only) paths_only=true ;;
        --require-tasks) require_tasks=true ;;
        --include-tasks) include_tasks=true ;;
    esac
    shift
done

branch_name=$(git rev-parse --abbrev-ref HEAD)
feature_dir="specs/$branch_name"
feature_spec="$feature_dir/spec.md"
feature_tasks="$feature_dir/tasks.md"

if [ ! -f "$feature_spec" ]; then
    echo "Error: Spec file not found at $feature_spec"
    exit 1
fi

if [ "$require_tasks" = true ] && [ ! -f "$feature_tasks" ]; then
    echo "Error: Tasks file not found at $feature_tasks"
    exit 1
fi

if [ "$json_output" = true ]; then
    if [ "$paths_only" = true ]; then
        echo "{\"FEATURE_DIR\":\"$feature_dir\",\"FEATURE_SPEC\":\"$feature_spec\"}"
    else
        docs_json="["
        for doc in "$feature_dir"/*.md;
do
            docs_json+=\""$doc\"","
        done
        docs_json=${docs_json%,}
        docs_json+="]"

        if [ "$include_tasks" = true ]; then
             echo "{\"FEATURE_DIR\":\"$feature_dir\",\"AVAILABLE_DOCS\":$docs_json, \"FEATURE_TASKS\":\"$feature_tasks\"}"
        else
            echo "{\"FEATURE_DIR\":\"$feature_dir\",\"AVAILABLE_DOCS\":$docs_json}"
        fi
    fi
fi


