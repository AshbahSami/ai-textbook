#!/bin/bash
# A simple script to create a new feature branch and spec file.
# This is a simplified version for demonstration purposes.

# Default values
number=""
short_name=""
description=""
json_output=false

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --number) number="$2"; shift ;;
        --short-name) short_name="$2"; shift ;;
        --json) json_output=true ;;
        *) description="$1" ;;
    esac
    shift
done

if [ -z "$number" ] || [ -z "$short_name" ]; then
    echo "Usage: $0 --number <number> --short-name <short-name> [--json] <description>"
    exit 1
fi

branch_name="${number}-${short_name}"
spec_dir="specs/${branch_name}"
spec_file="${spec_dir}/spec.md"

git checkout -b "$branch_name"
mkdir -p "$spec_dir"
echo "# Feature Specification: $description" > "$spec_file"
echo "" >> "$spec_file"
echo "Feature Branch: $branch_name" >> "$spec_file"

if [ "$json_output" = true ]; then
    echo "{\"BRANCH_NAME\":\"$branch_name\",\"SPEC_FILE\":\"$spec_file\"}"
fi

