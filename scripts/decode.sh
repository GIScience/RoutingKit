#!/bin/bash
decode="../bin/decode_vector" 
data_type=$1
shift
for i
do
  cmd="$decode $data_type $i"
  echo -n "$i: "
  $cmd
done

