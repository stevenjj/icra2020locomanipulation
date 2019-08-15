#!/bin/bash
num=10
parent_folder="param_set_$num/both_hands"

echo "Creating folders: $parent_folder/raw_positive_initial_config_data"
echo "Creating folders: $parent_folder/raw_positive_transitions_data"
echo "Creating folders: $parent_folder/transitions_data_with_task_space_info/negative_examples"
echo "Creating folders: $parent_folder/transitions_data_with_task_space_info/positive_examples"

mkdir -p "$parent_folder/raw_positive_initial_config_data"
mkdir -p "$parent_folder/raw_positive_transitions_data"
mkdir -p "$parent_folder/transitions_data_with_task_space_info/negative_examples"
mkdir -p "$parent_folder/transitions_data_with_task_space_info/positive_examples"
