#!/bin/bash

if [ $# -lt 1 ]; then
  echo "Usage: query_kb <filename>"
  exit 1
fi

ros2 service call /triplestar_kb/query triplestar_kb_msgs/srv/Query "{query_type: 1, query: '$(< "$1")'}"
