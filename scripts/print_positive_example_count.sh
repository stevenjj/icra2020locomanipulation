#!/bin/bash
num=1
parent_folder="/home/$USER/Data/param_set_$num"

count_data(){
	local folder_path=$1
	local filename_prefix=$2
	local count=`ls $folder_path | grep $filename_prefix | wc -l` 
	echo "  $filename_prefix count:" $count
}

echo "Number of Positive Examples Generated"

count_data "$parent_folder/right_hand/transitions_data_with_task_space_info/positive_examples" "right_hand_right_foot"
count_data "$parent_folder/right_hand/transitions_data_with_task_space_info/positive_examples" "right_hand_left_foot"

count_data "$parent_folder/left_hand/transitions_data_with_task_space_info/positive_examples" "left_hand_right_foot"
count_data "$parent_folder/left_hand/transitions_data_with_task_space_info/positive_examples" "left_hand_left_foot"

count_data "$parent_folder/both_hands/transitions_data_with_task_space_info/positive_examples" "both_hands_right_foot"
count_data "$parent_folder/both_hands/transitions_data_with_task_space_info/positive_examples" "both_hands_left_foot"


echo " "
echo "Number of Negative Examples Generated"

count_data "$parent_folder/right_hand/transitions_data_with_task_space_info/negative_examples" "right_hand_right_foot"
count_data "$parent_folder/right_hand/transitions_data_with_task_space_info/negative_examples" "right_hand_left_foot"

count_data "$parent_folder/left_hand/transitions_data_with_task_space_info/negative_examples" "left_hand_right_foot"
count_data "$parent_folder/left_hand/transitions_data_with_task_space_info/negative_examples" "left_hand_left_foot"

count_data "$parent_folder/both_hands/transitions_data_with_task_space_info/negative_examples" "both_hands_right_foot"
count_data "$parent_folder/both_hands/transitions_data_with_task_space_info/negative_examples" "both_hands_left_foot"

echo " "
echo " Dataset file size: " `du -hs $parent_folder`

