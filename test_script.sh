#!/bin/bash
# A simple script to check for the existence of a feature spec file.
# This is a simplified version for demonstration purposes.

branch_name=$(git rev-parse --abbrev-ref HEAD)
feature_dir="specs/$branch_name"

for doc in "$feature_dir"/*.md; do
    echo "$doc"
done
