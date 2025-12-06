#!/bin/bash
# A simple script to create a PHR.
# This is a simplified version for demonstration purposes.

source .specify/scripts/bash/common.sh

title=""
stage=""
feature=""
json_output=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --title) title="$2"; shift ;;
        --stage) stage="$2"; shift ;;
        --feature) feature="$2"; shift ;;
        --json) json_output=true ;;
    esac
    shift
done

if [ -z "$title" ] || [ -z "$stage" ]; then
    echo "Usage: $0 --title <title> --stage <stage> [--feature <feature>] [--json]"
    exit 1
fi

# Basic slugify
slug=$(echo "$title" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]+/ /g' | sed 's/^ //g;s/ $//g' | tr ' ' '-')

# Simple ID generation (not robust, just for demo)
phr_id="0001" # Start with 0001, in a real scenario this would increment

phr_dir="history/prompts/$stage"
if [ "$feature" != "" ]; then
    phr_dir="history/prompts/$feature"
fi
mkdir -p "$phr_dir"

phr_file="$phr_dir/$phr_id-$slug.$stage.prompt.md"

# Read template
template_content=$(cat .specify/templates/phr-template.prompt.md)

# Fill template
parsed_content=$(parse_phr_template \
    "$template_content" \
    "$phr_id" \
    "$title" \
    "$stage" \
    "$feature" \
    "{{PROMPT_TEXT}}" \
    "{{RESPONSE_TEXT}}" \
    "{{FILES_YAML}}" \
    "{{TESTS_YAML}}" \
)

echo "$parsed_content" > "$phr_file"

if [ "$json_output" = true ]; then
    echo "{\"id\":\"$phr_id\",\"path\":\"$phr_file\",\"context\":\"$stage\",\"stage\":\"$stage\",\"feature\":\"$feature\",\"template\":\"phr-template.prompt.md\"}"
fi