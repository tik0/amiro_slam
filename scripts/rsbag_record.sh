#/bin/bash

V="0.16"

rsbagcl${V} record \
  --idl-path "/usr/share/rst${V}/proto/stable/" \
  --load-idl "/usr/share/rst${V}/proto/stable/rst/vision/LaserScan.proto" \
  --load-idl "/usr/share/rst${V}/proto/stable/rst/geometry/Pose.proto" \
  -o tmp.tide \
  'spread:/tracking/merger' \
  'spread:/amiro1/'
