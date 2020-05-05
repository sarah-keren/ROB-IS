#!/bin/bash

# Usage: ./generate_objects.sh MAP INIT_DOUGHNUTS FINAL_DOUGHNUTS VARIANTS BASE_NAME
# Example ./generate_objects.sh $(rospack find rosplan_stage_demo)/maps/lt13.yaml 1 80 10 lt13
if [ $# -lt 5 ]
  then
    echo "Error: not enough arguments supplied"
    echo "Usage: ./generate_problems.sh MAP INIT_DOUGHNUTS FINAL_DOUGHNUTS VARIANTS BASE_NAME"
    exit 1
fi

for (( i=$2; i<=$3; i++ )); do
	printf -v ii "%02d" $i;
	for (( v=0; v<=$4; v++ )); do
		printf -v vv "%02d" $v;
		fname="${5}_${ii}doughnuts_variant${vv}.yaml";
		echo $fname
		python $(rospack find hidden_costs_generator)/src/HiddenCostsGenerator.py $1 $fname $i 0 &> /dev/null
	done
done