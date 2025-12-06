#!/bin/bash
# Common utility functions for specify scripts.

# Function to extract a value from a JSON string.
# Usage: get_json_value "$json_string" "$key"
get_json_value() {
  echo "$1" | python3 -c "import json,sys; print(json.load(sys.stdin).get('$2', ''))"
}

# Function to get the current feature branch name.
get_feature_branch() {
  git rev-parse --abbrev-ref HEAD
}

# Function to get the feature directory.
get_feature_dir() {
  local branch_name=$(get_feature_branch)
  echo "specs/$branch_name"
}

# Function to get the path to the spec file.
get_feature_spec_file() {
  local feature_dir=$(get_feature_dir)
  echo "$feature_dir/spec.md"
}

# Function to get the path to the plan file.
get_feature_plan_file() {
  local feature_dir=$(get_feature_dir)
  echo "$feature_dir/plan.md"
}

# Function to get the path to the tasks file.
get_feature_tasks_file() {
  local feature_dir=$(get_feature_dir)
  echo "$feature_dir/tasks.md"
}

# Function to parse a PHR template and fill placeholders.
# Usage: parse_phr_template "$template_content" "$id" "$title" "$stage" "$feature" ...
parse_phr_template() {
  local template_content="$1"
  shift
  local phr_id="$1"
  shift
  local phr_title="$1"
  shift
  local phr_stage="$1"
  shift
  local phr_feature="$1"
  shift
  local phr_prompt_text="$1"
  shift
  local phr_response_text="$1"
  shift
  local phr_files_yaml="$1"
  shift
  local phr_tests_yaml="$1"
  shift

  local current_date=$(date -I)
  local current_branch=$(get_feature_branch)
  local model_name="gemini-pro" # Assuming gemini-pro as the model

  local parsed_content="${template_content//\{\{ID\}\}/$phr_id}"
  parsed_content="${parsed_content//\{\{TITLE\}\}/$phr_title}"
  parsed_content="${parsed_content//\{\{STAGE\}\}/$phr_stage}"
  parsed_content="${parsed_content//\{\{DATE_ISO\}\}/$current_date}"
  parsed_content="${parsed_content//\{\{SURFACE\}\}/agent}"
  parsed_content="${parsed_content//\{\{MODEL\}\}/$model_name}"
  parsed_content="${parsed_content//\{\{FEATURE\}\}/$phr_feature}"
  parsed_content="${parsed_content//\{\{BRANCH\}\}/$current_branch}"
  parsed_content="${parsed_content//\{\{USER\}\}/unspecified}" # Assuming user is unspecified
  parsed_content="${parsed_content//\{\{COMMAND\}\}/$0}" # Current script name
  parsed_content="${parsed_content//\{\{LABELS\}\}/}" # Empty labels for now
  parsed_content="${parsed_content//\{\{LINKS_SPEC\}\}/null}"
  parsed_content="${parsed_content//\{\{LINKS_TICKET\}\}/null}"
  parsed_content="${parsed_content//\{\{LINKS_ADR\}\}/null}"
  parsed_content="${parsed_content//\{\{LINKS_PR\}\}/null}"
  parsed_content="${parsed_content//\{\{FILES_YAML\}\}/$phr_files_yaml}"
  parsed_content="${parsed_content//\{\{TESTS_YAML\}\}/$phr_tests_yaml}"
  parsed_content="${parsed_content//\{\{PROMPT_TEXT\}\}/$phr_prompt_text}"
  parsed_content="${parsed_content//\{\{RESPONSE_TEXT\}\}/$phr_response_text}"

  echo "$parsed_content"
}
