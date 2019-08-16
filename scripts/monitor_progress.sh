#!/bin/bash
rh_rf_count=`ls | grep right_hand_right_foot | wc -l`
echo "right_hand_right_foot count:" $rh_rf_count

rh_lf_count=`ls | grep right_hand_left_foot | wc -l`
echo "right_hand_left_foot count:" $rh_lf_count