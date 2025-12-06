#!/bin/bash
# A simple script to set up the plan file.
# This is a simplified version for demonstration purposes.

json_output=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --json) json_output=true ;;
    esac
    shift
done

branch_name=$(git rev-parse --abbrev-ref HEAD)
specs_dir="specs"
feature_spec="$specs_dir/$branch_name/spec.md"
impl_plan="$specs_dir/$branch_name/plan.md"

if [ ! -f "$feature_spec" ]; then
    echo "Error: Spec file not found at $feature_spec"
    exit 1
fi

cp .specify/templates/plan-template.md "$impl_plan"

if [ "$json_output" = true ]; then
    echo "{\"FEATURE_SPEC\":\"$feature_spec\",\"IMPL_PLAN\":\"$impl_plan\",\"SPECS_DIR\":\"$specs_dir\",\"BRANCH\":\"$branch_name\"}"
fi