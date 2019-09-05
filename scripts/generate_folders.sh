#!/bin/bash
num=1
parent_folder="/home/$USER/Data/param_set_$num"
planner_data_parent_folder="/home/$USER/Data/planner_data"

create_subfolders(){
	echo "Creating folders: $1/$2/raw_positive_initial_config_data"
	echo "Creating folders: $1/$2/raw_positive_transitions_data"
	echo "Creating folders: $1/$2/transitions_data_with_task_space_info/negative_examples"
	echo "Creating folders: $1/$2/transitions_data_with_task_space_info/positive_examples"

	mkdir -p "$1/$2/raw_positive_initial_config_data"
	mkdir -p "$1/$2/raw_positive_transitions_data"
	mkdir -p "$1/$2/transitions_data_with_task_space_info/negative_examples"
	mkdir -p "$1/$2/transitions_data_with_task_space_info/positive_examples"
}


create_subfolders $parent_folder "both_hands"
create_subfolders $parent_folder "right_hand"
create_subfolders $parent_folder "left_hand"

create_subfolders $planner_data_parent_folder "both_hands"
create_subfolders $planner_data_parent_folder "right_hand"
create_subfolders $planner_data_parent_folder "left_hand"

