#!/bin/sh

echo "syncing pointclouds..."
echo "destination: $1"
pwd

chmod 600 $1'/../read_only_rsync'
[ -d $1 ] || mkdir $1

rsync -ruzlve 'ssh -p9418 -i '$1'/../read_only_rsync -o IdentitiesOnly=yes' perception-data@team.suturo.de:/home/perception-data/data/pointclouds/* $1